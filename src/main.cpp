// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include <Arduino.h>
#include "science.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int8.h>

#define PIN_PH 14
#define PIN_MOISTURE 26
#define PIN_TDS 27
#define PIN_ORP 15
#define PIN_TEMP_SOIL 2

// #define PIN_PUMP_PWM 2
// #define PIN_PUMP_INA 2
// #define PIN_PUMP_INB 2

// Pump pump(PIN_PUMP_PWM, PIN_PUMP_INA, PIN_PUMP_INB);
pHSensor ph_sensor(PIN_PH);
CapacitiveMoistureSensor moisture_sensor(PIN_MOISTURE);
TDSSensor tds_sensor(PIN_TDS);
ORPSensor orp_sensor(PIN_ORP);
DS18B20Sensor soil_temp_sensor(PIN_TEMP_SOIL);
BME688Sensor bme_sensor;
SCD41Sensor scd_sensor;

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
  IDX_PH = 0,
  IDX_SOIL_MOISTURE,
  IDX_TDS,
  IDX_ORP,
  IDX_SOIL_TEMP,
  IDX_BME_TEMP,
  IDX_BME_HUM,
  IDX_BME_PRESS,
  IDX_BME_GAS,
  IDX_SCD_CO2,
  TELEMETRY_SIZE
};

float msg_data_buffer[TELEMETRY_SIZE];
uint8_t state_machine_tick = 0;

// async wait trackers for slow digital sensors
uint8_t temp_wait_ticks = 0;
uint8_t scd_wait_ticks = 0;

void sensor_cmd_callback(const void *msgin)
{
  const auto *msg = static_cast<const std_msgs__msg__UInt8 *>(msgin);

  // Example: Command 62 -> sensor_id = 6 (Temp), action = 2 (Read)
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
      temp_wait_ticks = 8;
      break;
    case IDX_BME_TEMP:
      bme_sensor.get_data(
          telemetry_msg.data.data[IDX_BME_TEMP],
          telemetry_msg.data.data[IDX_BME_HUM],
          telemetry_msg.data.data[IDX_BME_PRESS],
          telemetry_msg.data.data[IDX_BME_GAS]);
      break;
    case IDX_SCD_CO2:
      scd_sensor.request_read();
      scd_wait_ticks = 2;
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
      telemetry_msg.data.data[IDX_SOIL_TEMP] = soil_temp_sensor.get_value();
    }
  }

  // check if the SCD41 CO2 bus preparation (200ms) is finished
  if (scd_wait_ticks > 0)
  {
    scd_wait_ticks--;
    if (scd_wait_ticks == 0)
    {
      telemetry_msg.data.data[IDX_SCD_CO2] = scd_sensor.get_value();
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
  // Initialize the array values to exactly 0.0f on boot
  for (int i = 0; i < TELEMETRY_SIZE; i++)
  {
    msg_data_buffer[i] = 0.0f;
  }

  // Sensors are NO LONGER initialized here.
  // They wait for the ACTION 1 command over ROS 2.
  // pump.init_pump();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "science_module_node", "", &support);

  telemetry_msg.data.capacity = TELEMETRY_SIZE;
  telemetry_msg.data.size = TELEMETRY_SIZE;
  telemetry_msg.data.data = msg_data_buffer;

  // Initialize Publisher
  rclc_publisher_init_default(
      &telemetry_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/science/telemetry");

  // Initialize Command Subscriber
  rclc_subscription_init_default(
      &sensor_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/science/sensor_cmd");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &sensor_cmd_sub, &sensor_cmd_msg, &sensor_cmd_callback, ON_NEW_DATA);
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
