// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef DRILL_H
#define DRILL_H

#include <Arduino.h>
#include <Servo.h>
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
  int m_cache_speed;
  bool m_cache_dir;
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

// Servo Class ====================================================================================================
class CustomServo
{
public:
  CustomServo(uint8_t servo_pin);
  void init();
  void set_target(float target_normalized);
  void update();

private:
  int normalized_to_us(float t);

  uint8_t m_servo_pin;
  Servo m_servo;

  float m_current_normalized;
  float m_target_normalized;
  uint32_t m_last_update_ms;

  // Use static constexpr to compile these directly into flash, saving RAM
  static constexpr int m_min_us = 850;
  static constexpr int m_max_us = 2200;
  static constexpr float m_slew_deg_per_sec = 550.0f;
};

// Switch Class ====================================================================================================
class LimitSwitch
{
public:
  LimitSwitch(uint8_t pin_switch, uint32_t debounce_ms);
  void init();
  bool is_triggered();
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
  bool m_is_stable;

  uint32_t m_last_read_ms;
  static constexpr uint8_t STABILITY_BUFFER_SIZE = 5;
  float m_recent_reads[STABILITY_BUFFER_SIZE];
  uint8_t m_buffer_idx;
};

#endif
