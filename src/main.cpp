// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include <Arduino.h>
#include "science.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

#define PIN_PH 14
#define PIN_MOISTURE 26
#define PIN_TDS 27
#define PIN_ORP 15
#define PIN_TEMP_SOIL 2

pHSensor ph_sensor(PIN_PH);
CapacitiveMoistureSensor moisture_sensor(PIN_MOISTURE);
TDSSensor tds_sensor(PIN_TDS);
ORPSensor orp_sensor(PIN_ORP, 531.612976);
DS18B20Sensor soil_temp_sensor(PIN_TEMP_SOIL);
BME688Sensor bme_sensor;
SCD41Sensor scd_sensor; // FIX: no address arg anymore - the SparkFun library owns that (fixed at 0x62)

enum MicroROSState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
MicroROSState uros_state = WAITING_AGENT;
uint8_t entities_stage = 0;

rcl_publisher_t telemetry_pub;
rcl_subscription_t sensor_cmd_sub;

std_msgs__msg__Float32MultiArray telemetry_msg;
std_msgs__msg__UInt8 sensor_cmd_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

enum TelemetryIndex
{
  IDX_PH = 0,            // 0.0 - 14.0 (7.0 is neutral)
  IDX_SOIL_MOISTURE = 1, // 0 - 100 %
  IDX_TDS = 2,           // ppm or µS/cm (Minerals/Salts)
  IDX_ORP = 3,           // mV (Oxidation-Reduction Potential)
  IDX_SOIL_TEMP = 4,     // °C (DS18B20 Soil Probe)
  IDX_BME_TEMP = 5,      // °C (Ambient Air)
  IDX_BME_HUM = 6,       // % (Relative Humidity)
  IDX_BME_PRESS = 7,     // hPa (Atmospheric Pressure)
  IDX_BME_GAS = 8,       // Ohms (VOC Resistance, higher = cleaner)
  IDX_SCD_CO2 = 9,       // ppm (Baseline fresh air is ~400)
  TELEMETRY_SIZE = 10
};

float msg_data_buffer[TELEMETRY_SIZE];
uint8_t state_machine_tick = 0;

uint8_t temp_wait_ticks = 0;
uint8_t temp_retry_count = 0;
uint8_t bme_wait_ticks = 0;

bool create_entities();
void destroy_entities();

void sensor_cmd_callback(const void *msgin)
{
  const auto *msg = static_cast<const std_msgs__msg__UInt8 *>(msgin);

  uint8_t sensor_id = msg->data / 10;
  uint8_t action = msg->data % 10;

  if (action == 1)
  {
    switch (sensor_id)
    {
    case IDX_PH:
      ph_sensor.init();
      break;
    case IDX_SOIL_MOISTURE:
      moisture_sensor.init();
      break;
    case IDX_TDS:
      tds_sensor.init();
      break;
    case IDX_ORP:
      orp_sensor.init();
      break;
    case IDX_SOIL_TEMP:
      soil_temp_sensor.init();
      break;
    case IDX_BME_TEMP:
      bme_sensor.init();
      break;
    case IDX_SCD_CO2:
      scd_sensor.init();
      break;
    }
  }
  else if (action == 2)
  {
    switch (sensor_id)
    {
    case IDX_PH:
      telemetry_msg.data.data[IDX_PH] = ph_sensor.get_value();
      break;
    case IDX_SOIL_MOISTURE:
      telemetry_msg.data.data[IDX_SOIL_MOISTURE] = moisture_sensor.get_value();
      break;
    case IDX_TDS:
      telemetry_msg.data.data[IDX_TDS] = tds_sensor.get_value();
      break;
    case IDX_ORP:
      telemetry_msg.data.data[IDX_ORP] = orp_sensor.get_value();
      break;
    case IDX_SOIL_TEMP:
      soil_temp_sensor.request_read();
      temp_wait_ticks = 9;  // a little margin over the 750ms max conversion time
      temp_retry_count = 0; // reset the retry budget on every fresh ROS-requested read
      break;
    case IDX_BME_TEMP:
      bme_sensor.request_read();
      bme_wait_ticks = 4; // ~400ms: covers oversampling16 x3 + 150ms gas heater soak
      break;
    case IDX_SCD_CO2:
      telemetry_msg.data.data[IDX_SCD_CO2] = scd_sensor.get_value();
      break;
    }
  }
}

void timer_callback(rcl_timer_t *timer_obj, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer_obj == NULL)
    return;

  // check if the DS18B20 Temp calculation (800ms) is finished
  if (temp_wait_ticks > 0)
  {
    temp_wait_ticks--;
    if (temp_wait_ticks == 0)
    {
      float temp = soil_temp_sensor.get_value();

      // FIX: a lone -127 is very often a transient OneWire glitch (missing
      // conversion, corrupted scratchpad CRC) rather than a genuinely
      // disconnected sensor - give it one automatic retry before reporting
      // a failed reading to ROS. Deliberately NOT retrying on 0.0, since
      // that's a value the probe could legitimately report.
      if (temp == -127.0f && temp_retry_count < 1)
      {
        temp_retry_count++;
        soil_temp_sensor.request_read();
        temp_wait_ticks = 9;
      }
      else
      {
        telemetry_msg.data.data[IDX_SOIL_TEMP] = temp;
      }
    }
  }

  // check if the BME688 forced-mode conversion is finished
  if (bme_wait_ticks > 0)
  {
    bme_wait_ticks--;
    if (bme_wait_ticks == 0)
    {
      bme_sensor.get_data(
          telemetry_msg.data.data[IDX_BME_TEMP],
          telemetry_msg.data.data[IDX_BME_HUM],
          telemetry_msg.data.data[IDX_BME_PRESS],
          telemetry_msg.data.data[IDX_BME_GAS]);
    }
  }

  // publish the array at exactly 1 Hz (every 10 ticks of the 100ms timer)
  if (state_machine_tick >= 9)
  {
    rcl_publish(&telemetry_pub, &telemetry_msg, NULL);
    state_machine_tick = 0;
  }
  else
  {
    state_machine_tick++;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();  // BME688 lives here (pins 18/19) - the Zanshin library is hardcoded to default Wire
  Wire1.begin(); // SCD41 lives here (pins 17/16) - confirm its SDA/SCL wires match these pins, not Wire's
  Wire2.begin(); // reserved / unused for now

  // Initialize the array values to exactly 0.0f on boot
  for (int i = 0; i < TELEMETRY_SIZE; i++)
  {
    msg_data_buffer[i] = 0.0f;
  }

  // Sensors are NOT initialized here - they wait for an ACTION 1 command over ROS 2.

  set_microros_serial_transports(Serial);
}

void loop()
{
  static uint32_t last_ping_ms = 0;
  uint32_t now = millis();

  switch (uros_state)
  {
  case WAITING_AGENT:
    if (now - last_ping_ms > 500)
    {
      last_ping_ms = now;
      uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
    }
    break;

  case AGENT_AVAILABLE:
    uros_state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
    if (uros_state == WAITING_AGENT)
      destroy_entities();
    break;

  case AGENT_CONNECTED:
    if (now - last_ping_ms > 500)
    {
      last_ping_ms = now;
      uros_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 2)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    }

    if (uros_state == AGENT_CONNECTED)
    {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    set_microros_serial_transports(Serial);
    uros_state = WAITING_AGENT;
    break;
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  entities_stage = 0;

  if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator))
    return false;
  entities_stage = 1;

  if (RCL_RET_OK != rclc_node_init_default(&node, "science_module_node", "", &support))
    return false;
  entities_stage = 2;

  telemetry_msg.data.capacity = TELEMETRY_SIZE;
  telemetry_msg.data.size = TELEMETRY_SIZE;
  telemetry_msg.data.data = msg_data_buffer;

  if (RCL_RET_OK != rclc_publisher_init_default(
                        &telemetry_pub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                        "/science/telemetry"))
    return false;
  entities_stage = 3;

  if (RCL_RET_OK != rclc_subscription_init_default(
                        &sensor_cmd_sub, &node,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                        "/science/sensor_cmd"))
    return false;
  entities_stage = 4;

  if (RCL_RET_OK != rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback))
    return false;
  entities_stage = 5;

  if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 2, &allocator))
    return false;
  entities_stage = 6;

  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &sensor_cmd_sub, &sensor_cmd_msg, &sensor_cmd_callback, ON_NEW_DATA);

  return true;
}

void destroy_entities()
{
  if (entities_stage == 0)
    return;

  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  if (entities_stage >= 6)
    rclc_executor_fini(&executor);
  if (entities_stage >= 5)
    rcl_timer_fini(&timer);
  if (entities_stage >= 4)
    rcl_subscription_fini(&sensor_cmd_sub, &node);
  if (entities_stage >= 3)
    rcl_publisher_fini(&telemetry_pub, &node);
  if (entities_stage >= 2)
    rcl_node_fini(&node);
  if (entities_stage >= 1)
    rclc_support_fini(&support);

  entities_stage = 0;
}
