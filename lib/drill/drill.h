// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef DRILL_H
#define DRILL_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <HX711.h>
#include <IntervalTimer.h>

#define LIMIT_SWITCH_INSTANCES 2

// Motor Classes ====================================================================================================
class Driver
{
public:
  Driver(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_driver();
  void drive(int pwm_speed, bool dir);
  void stop_driver();

private:
  uint8_t m_pin_pwm;
  uint8_t m_pin_in1;
  uint8_t m_pin_in2;
  int m_cache_speed = 0;
  bool m_cache_dir = false;
};

class AugerMotor : private Driver
{
public:
  AugerMotor(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_motor();
  void drive_motor(int pwm_speed, bool dir);
  void stop_motor();
};

class LeadScrewMotor : private Driver
{
public:
  LeadScrewMotor(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_motor();
  void drive_motor(int pwm_speed, bool dir);
  void stop_motor();
};

class LinearActuator : private Driver
{
public:
  LinearActuator(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_motor();
  void extend();
  void retract();
  void home(bool dir);
  void stop_motor();

  // functions in case we need to send modified values
  void extend(int pwm_speed, float req_ext);
  void retract(int pwm_speed, float req_ext);

private:
  const float m_oem_max_speed = 15.0; // [mm/s]
  const float m_oem_max_ext = 100.0;  // [mm]

  int m_pwm_speed = 255;                                        // 100% duty-cycle for max power
  float m_req_ext = 75.0;                                       // [mm]
  float m_pwm_dur = ((m_req_ext) / (m_oem_max_speed * 1));      // [s]
  float m_home_dur = ((m_oem_max_ext) / (m_oem_max_speed * 1)); // [s]

  IntervalTimer m_timer;
  static LinearActuator *m_instance_la;
  static void isr_timer_router();
  void handle_isr();
};

// Servo Classes ====================================================================================================

// Standard closed-loop positional servo (e.g. the gripper). Commanded with a
// normalized position in [0.0, 1.0]; internally holds that angle and slews
// smoothly toward a new target rather than snapping to it.
class GripperServo
{
public:
  GripperServo(uint8_t servo_pin, int min_us = 850, int max_us = 2200,
               float slew_deg_per_sec = 550.0f, float travel_deg = 270.0f);
  void init();
  void set_target(float target_normalized);
  void update();

  // Feedback
  float get_position() const { return m_current_normalized; }
  float get_target() const { return m_target_normalized; }
  bool is_at_target() const;

  // Control
  void stop();   // cancel any in-progress move, hold current position
  void detach(); // power down the servo (re-attach by calling init() again)

private:
  int normalized_to_us(float t) const;

  uint8_t m_servo_pin;
  Servo m_servo;

  float m_current_normalized;
  float m_target_normalized;
  uint32_t m_last_update_ms;
  bool m_initialized;

  const int m_min_us;
  const int m_max_us;
  const float m_slew_deg_per_sec;
  const float m_travel_deg;
};

// 360-degree continuous-rotation servo (e.g. the lid). Unlike GripperServo,
// pulse width here maps to SPEED and DIRECTION, not an absolute angle - there
// is no positional feedback at all, so this class has no concept of "target
// position", only "how fast and which way right now".
//
// neutral_us is the calibrated stop point for this specific physical servo
// unit. It varies unit-to-unit (often not exactly 1500us) because it depends
// on how the servo's internal centering pot was trimmed - test yours and
// tune this value until it truly holds still at speed 0.
// max_deviation_us is how far above/below neutral corresponds to full speed
// in each direction (typical hobby servos: ~350-500us for full speed).
class LidServo
{
public:
  LidServo(uint8_t servo_pin, int neutral_us = 1500, int max_deviation_us = 400);
  void init();
  void set_speed(float speed);
  void stop();
  float get_speed() const { return m_current_speed; }
  void rotate_for(float speed, float duration_s);

  // Call every loop() iteration. Auto-stops if no new set_speed() has
  // arrived within command_timeout_ms - unlike GripperServo, a speed
  // command has no natural resting state, so a dropped "stop" packet
  // (or a lost ROS link) would otherwise spin this servo forever.
  void update(uint32_t command_timeout_ms = 500);

private:
  uint8_t m_servo_pin;
  Servo m_servo;
  int m_neutral_us;
  int m_max_deviation_us;
  float m_current_speed;
  uint32_t m_last_cmd_ms = 0; // NEW

  IntervalTimer m_timer;
  static LidServo *m_instance_lid;
  static void isr_timer_router();
  void handle_isr();
};

// Switch Class ====================================================================================================
class LimitSwitch
{
public:
  LimitSwitch(uint8_t pin_switch, uint32_t debounce_ms);
  void init();

  // Edge-triggered, one-shot: returns true once per debounced trigger event,
  // then clears itself. Use this to react to a NEW trigger (e.g. stop a motor).
  bool is_triggered();

  // Level state: returns whether the switch is physically pressed RIGHT NOW,
  // with no side effects. Use this for telemetry/status publishing - reading
  // is_triggered() for that purpose would race with anything else consuming
  // the same one-shot flag and almost always read back false.
  bool is_pressed() const;

  volatile bool m_triggered;
  volatile uint32_t m_last_interrupt_time;

private:
  uint8_t m_pin_switch;
  uint32_t m_debounce_ms;

  void handle_isr();

  static LimitSwitch *m_instances_ls[LIMIT_SWITCH_INSTANCES];
  static uint8_t m_instance_count_ls;

  static void isr_router_ls_0();
  static void isr_router_ls_1();
};

// Load Cell Class ====================================================================================================
class LoadCell
{
public:
  LoadCell(uint8_t dout_pin, uint8_t sck_pin, float scale_factor = 1.0f);

  void init();

  // Calibration & Zeroing
  void tare_empty();
  void tare_with_lid();
  void set_scale(float scale_factor);
  float get_scale();

  // Data Acquisition
  bool is_ready();
  bool is_stable();
  float get_soil_weight();
  float get_raw_value(uint8_t samples = 1);

  void update();

private:
  uint8_t m_dout_pin;
  uint8_t m_sck_pin;
  float m_scale_factor;

  HX711 m_load_cell;

  float m_current_weight;
  float m_lid_tare_weight; // Stores the isolated weight of the lid
  float m_empty_tare_weight;
  bool m_is_stable;

  uint32_t m_last_read_ms;
  static constexpr uint8_t STABILITY_BUFFER_SIZE = 5;
  float m_recent_reads[STABILITY_BUFFER_SIZE];
  uint8_t m_buffer_idx;
};

class Pump : private Driver
{
public:
  Pump(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_motor();
  void release();
  void draw();
  void home(bool dir);
  void stop_motor();

  // functions in case we need to send modified values
  void release(int pwm_flowrate, float req_vol);
  void draw(int pwm_flowrate, float req_vol);

private:
  // average of 3 readings of liquid volume for time stamps is taken to get the flow rate
  const float m_oem_max_flowrate = (((100.0f / 15.0f) + (150.0f / 22.0f) + (200.0f / 29.5f)) / 3.0f); // [mL/s]
  // m_oem_max_flowrate = 6.755 mL/s

  float m_pwm_flowrate = 255;                                 // 100% duty-cycle for max power
  float m_req_vol = 75;                                       // [mL]
  float m_pwm_dur = ((m_req_vol) / (m_oem_max_flowrate * 1)); // [s]
  float m_home_dur = (100.0f / m_oem_max_flowrate);           // [s] for tube length of 100mL volume

  IntervalTimer m_timer;
  static Pump *m_instance_la;
  static void isr_timer_router();
  void handle_isr();
};

#endif