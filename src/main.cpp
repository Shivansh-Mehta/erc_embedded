// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "drill.h"
#include "emg.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
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

#define MOTOR1_PWM 15
#define MOTOR1_INA 41
#define MOTOR1_INB 40

#define MOTOR2_PWM 22
#define MOTOR2_INA 19
#define MOTOR2_INB 18

#define LINACT_PWM 28
#define LINACT_INA 30
#define LINACT_INB 29

#define LIMIT_SWITCH1 4
#define LIMIT_SWITCH2 5

#define SAND_BOX_DT 17
#define SAND_BOX_SCK 16
#define SAND_BOX_SF 1.0f
// #define SAND_BOX_SF -40.0f

#define ROCK_BOX_DT 34
#define ROCK_BOX_SCK 33
#define ROCK_BOX_SF 1.0f

#define DCONT_BOX_DT 32
#define DCONT_BOX_SCK 31
#define DCONT_BOX_SF 1.0f

#define STALIG_G 37
#define STALIG_Y 36
#define STALIG_R 35

#define GRIPPER_SERVO 23
#define LID_SERVO 10

enum MicroROSState
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
MicroROSState uros_state = WAITING_AGENT;
uint8_t entities_stage = 0;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor Subscriptions & Objects
rcl_subscription_t motor1_cmd_sub;
std_msgs__msg__Int32 motor1_pwm_cmd_msg;
AugerMotor motor1(MOTOR1_PWM, MOTOR1_INA, MOTOR1_INB);

rcl_subscription_t motor2_cmd_sub;
std_msgs__msg__Int32 motor2_pwm_cmd_msg;
LeadScrewMotor motor2(MOTOR2_PWM, MOTOR2_INA, MOTOR2_INB);

rcl_subscription_t linact_state_cmd_sub;
std_msgs__msg__UInt8 linact_state_cmd_msg;
rcl_subscription_t linact_cext_cmd_sub;
std_msgs__msg__Float32 linact_cext_cmd_msg;
LinearActuator linact(LINACT_PWM, LINACT_INA, LINACT_INB);

// Stacklight & Servos
rcl_subscription_t stalig_cmd_sub;
std_msgs__msg__UInt8 stalig_state_cmd_msg;
StackLight stalig(STALIG_G, STALIG_Y, STALIG_R);

rcl_subscription_t gservo_cmd_sub;
std_msgs__msg__Float32 gservo_cmd_msg;
CustomServo gservo(GRIPPER_SERVO);

rcl_subscription_t lservo_cmd_sub;
std_msgs__msg__Float32 lservo_cmd_msg;
CustomServo lservo(LID_SERVO);

// Load Cells (Sand, Rock, Drill Container)
rcl_subscription_t sand_box_cmd_sub;
std_msgs__msg__UInt8 sand_box_cmd_msg;
rcl_publisher_t sand_box_pub;
std_msgs__msg__Float32 sand_box_rec_msg;
LoadCell sand_box(SAND_BOX_DT, SAND_BOX_SCK, SAND_BOX_SF);

rcl_subscription_t rock_box_cmd_sub;
std_msgs__msg__UInt8 rock_box_cmd_msg;
rcl_publisher_t rock_box_pub;
std_msgs__msg__Float32 rock_box_rec_msg;
LoadCell rock_box(ROCK_BOX_DT, ROCK_BOX_SCK, ROCK_BOX_SF);

rcl_subscription_t drill_cont_cmd_sub;
std_msgs__msg__UInt8 drill_cont_cmd_msg;
rcl_publisher_t drill_cont_pub;
std_msgs__msg__Float32 drill_cont_rec_msg;
LoadCell drill_cont(DCONT_BOX_DT, DCONT_BOX_SCK, DCONT_BOX_SF);

rcl_publisher_t limit_switch_1_pub;
std_msgs__msg__UInt8 limit_switch_1_status;
LimitSwitch switch1(LIMIT_SWITCH1, 50);

rcl_publisher_t limit_switch_2_pub;
std_msgs__msg__UInt8 limit_switch_2_status;
LimitSwitch switch2(LIMIT_SWITCH2, 50);

bool create_entities();
void destroy_entities();

void motor1_cmd_callback(const void *msin);
void motor2_cmd_callback(const void *msin);
void linact_state_cmd_callback(const void *msin);
void linact_cext_cmd_callback(const void *msin);
void stalig_state_cmd_callback(const void *msin);
void gservo_cmd_callback(const void *msin);
void lservo_cmd_callback(const void *msin);
void sand_box_cmd_callback(const void *msin);
void rock_box_cmd_callback(const void *msin);
void dcont_box_cmd_callback(const void *msin);

void setup()
{
  switch1.init();
  switch2.init();
  motor1.init_motor();
  motor2.init_motor();
  linact.init_motor();
  stalig.init_light();
  gservo.init();
  lservo.init();
  sand_box.init();
  rock_box.init();
  drill_cont.init();

  set_microros_serial_transports(Serial);
}

void loop()
{
  if (switch1.is_triggered())
  {
    motor1.stop_motor();
    // motor2.stop_motor();
  }
  if (switch2.is_triggered())
  {
    motor1.stop_motor();
    // motor2.stop_motor();
  }

  // Non-blocking updates
  gservo.update();
  lservo.update();
  sand_box.update();
  rock_box.update();
  drill_cont.update();

  static uint32_t last_ping_ms = 0;
  static uint32_t last_pub_ms = 0;
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

      // Publish soil weights at 5Hz when stable
      if (now - last_pub_ms >= 200)
      {
        last_pub_ms = now;

        sand_box_rec_msg.data = sand_box.get_soil_weight();
        rcl_publish(&sand_box_pub, &sand_box_rec_msg, NULL);
        rock_box_rec_msg.data = rock_box.get_soil_weight();
        rcl_publish(&rock_box_pub, &rock_box_rec_msg, NULL);
        drill_cont_rec_msg.data = drill_cont.get_soil_weight();
        rcl_publish(&drill_cont_pub, &drill_cont_rec_msg, NULL);

        // if (sand_box.is_stable())
        // {
        //   sand_box_rec_msg.data = sand_box.get_soil_weight();
        //   rcl_publish(&sand_box_pub, &sand_box_rec_msg, NULL);
        // }
        // if (rock_box.is_stable())
        // {
        //   rock_box_rec_msg.data = rock_box.get_soil_weight();
        //   rcl_publish(&rock_box_pub, &rock_box_rec_msg, NULL);
        // }
        // if (drill_cont.is_stable())
        // {
        //   drill_cont_rec_msg.data = drill_cont.get_soil_weight();
        //   rcl_publish(&drill_cont_pub, &drill_cont_rec_msg, NULL);
        // }
        limit_switch_1_status.data = switch1.m_triggered;
        rcl_publish(&limit_switch_1_pub, &limit_switch_1_status, NULL);
        limit_switch_2_status.data = switch2.m_triggered;
        rcl_publish(&limit_switch_2_pub, &limit_switch_2_status, NULL);
      }
    }
    break;

  case AGENT_DISCONNECTED:
    destroy_entities();
    set_microros_serial_transports(Serial);
    uros_state = WAITING_AGENT;
    break;
  }
}

// Callbacks
void motor1_cmd_callback(const void *msin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msin;
  int target_speed = msg->data;
  bool dir = (target_speed >= 0);
  motor1.drive_motor(abs(target_speed), dir);
}

void motor2_cmd_callback(const void *msin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msin;
  int target_speed = msg->data;
  bool dir = (target_speed >= 0);
  motor2.drive_motor(abs(target_speed), dir);
}

void linact_state_cmd_callback(const void *msin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msin;
  uint8_t state = msg->data;
  if (state == 1)
    linact.extend();
  else if (state == 2)
    linact.retract();
  else if (state == 3)
    linact.home(true);
  else if (state == 4)
    linact.home(false);
}

void linact_cext_cmd_callback(const void *msin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msin;
  float cext = msg->data;
  if (cext > 0)
    linact.extend(255, cext);
  else
    linact.retract(255, abs(cext));
}

void stalig_state_cmd_callback(const void *msin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msin;
  stalig.state(msg->data);
}

void gservo_cmd_callback(const void *msin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msin;
  gservo.set_target(msg->data);
}

void lservo_cmd_callback(const void *msin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msin;
  lservo.set_target(msg->data);
}

void sand_box_cmd_callback(const void *msin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msin;
  if (msg->data == 1)
    sand_box.tare_empty();
  else if (msg->data == 2)
    sand_box.tare_with_lid();
}

void rock_box_cmd_callback(const void *msin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msin;
  if (msg->data == 1)
    rock_box.tare_empty();
  else if (msg->data == 2)
    rock_box.tare_with_lid();
}

void dcont_box_cmd_callback(const void *msin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msin;
  if (msg->data == 1)
    drill_cont.tare_empty();
  else if (msg->data == 2)
    drill_cont.tare_with_lid();
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();
  entities_stage = 0;

  if (RCL_RET_OK != rclc_support_init(&support, 0, NULL, &allocator))
    return false;
  entities_stage = 1;

  if (RCL_RET_OK != rclc_node_init_default(&node, "teensy_drill_node", "", &support))
    return false;
  entities_stage = 2;

  // Subscriptions
  if (RCL_RET_OK != rclc_subscription_init_default(&motor1_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor1/cmd_speed"))
    return false;
  entities_stage = 3;

  if (RCL_RET_OK != rclc_subscription_init_default(&motor2_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor2/cmd_speed"))
    return false;
  entities_stage = 4;

  if (RCL_RET_OK != rclc_subscription_init_default(&linact_state_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "linact/state"))
    return false;
  entities_stage = 5;

  if (RCL_RET_OK != rclc_subscription_init_default(&linact_cext_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "linact/cext"))
    return false;
  entities_stage = 6;

  if (RCL_RET_OK != rclc_subscription_init_default(&stalig_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "stalig/state"))
    return false;
  entities_stage = 7;

  if (RCL_RET_OK != rclc_subscription_init_default(&gservo_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "gservo/state"))
    return false;
  entities_stage = 8;

  if (RCL_RET_OK != rclc_subscription_init_default(&lservo_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "lservo/state"))
    return false;
  entities_stage = 9;

  if (RCL_RET_OK != rclc_subscription_init_default(&sand_box_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "sand_box/tare"))
    return false;
  entities_stage = 10;

  if (RCL_RET_OK != rclc_subscription_init_default(&rock_box_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "rock_box/tare"))
    return false;
  entities_stage = 11;

  if (RCL_RET_OK != rclc_subscription_init_default(&drill_cont_cmd_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "drill_cont/tare"))
    return false;
  entities_stage = 12;

  // Publishers
  if (RCL_RET_OK != rclc_publisher_init_default(&sand_box_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "sand_box/weight"))
    return false;
  entities_stage = 13;

  if (RCL_RET_OK != rclc_publisher_init_default(&rock_box_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "rock_box/weight"))
    return false;
  entities_stage = 14;

  if (RCL_RET_OK != rclc_publisher_init_default(&drill_cont_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "drill_cont/weight"))
    return false;
  entities_stage = 15;

  if (RCL_RET_OK != rclc_publisher_init_default(&limit_switch_1_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "ls1/status"))
    return false;
  entities_stage = 16;

  if (RCL_RET_OK != rclc_publisher_init_default(&limit_switch_2_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "ls2/status"))
    return false;
  entities_stage = 17;

  // Initialize Executor for 10 subscription handles
  if (RCL_RET_OK != rclc_executor_init(&executor, &support.context, 12, &allocator))
    return false;
  entities_stage = 18;

  // Add subscriptions to executor
  rclc_executor_add_subscription(&executor, &motor1_cmd_sub, &motor1_pwm_cmd_msg, &motor1_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &motor2_cmd_sub, &motor2_pwm_cmd_msg, &motor2_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &linact_state_cmd_sub, &linact_state_cmd_msg, &linact_state_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &linact_cext_cmd_sub, &linact_cext_cmd_msg, &linact_cext_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &stalig_cmd_sub, &stalig_state_cmd_msg, &stalig_state_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &gservo_cmd_sub, &gservo_cmd_msg, &gservo_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &lservo_cmd_sub, &lservo_cmd_msg, &lservo_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sand_box_cmd_sub, &sand_box_cmd_msg, &sand_box_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &rock_box_cmd_sub, &rock_box_cmd_msg, &rock_box_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &drill_cont_cmd_sub, &drill_cont_cmd_msg, &dcont_box_cmd_callback, ON_NEW_DATA);

  return true;
}

void destroy_entities()
{
  if (entities_stage == 0)
    return;

  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  if (entities_stage >= 18)
    rclc_executor_fini(&executor);
  if (entities_stage >= 17)
    rcl_publisher_fini(&limit_switch_1_pub, &node);
  if (entities_stage >= 16)
    rcl_publisher_fini(&limit_switch_2_pub, &node);
  if (entities_stage >= 15)
    rcl_publisher_fini(&drill_cont_pub, &node);
  if (entities_stage >= 14)
    rcl_publisher_fini(&rock_box_pub, &node);
  if (entities_stage >= 13)
    rcl_publisher_fini(&sand_box_pub, &node);
  if (entities_stage >= 12)
    rcl_subscription_fini(&drill_cont_cmd_sub, &node);
  if (entities_stage >= 11)
    rcl_subscription_fini(&rock_box_cmd_sub, &node);
  if (entities_stage >= 10)
    rcl_subscription_fini(&sand_box_cmd_sub, &node);
  if (entities_stage >= 9)
    rcl_subscription_fini(&lservo_cmd_sub, &node);
  if (entities_stage >= 8)
    rcl_subscription_fini(&gservo_cmd_sub, &node);
  if (entities_stage >= 7)
    rcl_subscription_fini(&stalig_cmd_sub, &node);
  if (entities_stage >= 6)
    rcl_subscription_fini(&linact_cext_cmd_sub, &node);
  if (entities_stage >= 5)
    rcl_subscription_fini(&linact_state_cmd_sub, &node);
  if (entities_stage >= 4)
    rcl_subscription_fini(&motor2_cmd_sub, &node);
  if (entities_stage >= 3)
    rcl_subscription_fini(&motor1_cmd_sub, &node);
  if (entities_stage >= 2)
    rcl_node_fini(&node);
  if (entities_stage >= 1)
    rclc_support_fini(&support);

  entities_stage = 0;
}
