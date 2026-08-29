// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include "drill.h"

Driver::Driver(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : m_pin_pwm(pin_pwm),
      m_pin_in1(pin_in1),
      m_pin_in2(pin_in2) {};

void Driver::init_driver()
{
  analogWriteFrequency(m_pin_pwm, 10000);
  pinMode(m_pin_pwm, OUTPUT);
  pinMode(m_pin_in1, OUTPUT);
  pinMode(m_pin_in2, OUTPUT);
  stop_driver();
}

void Driver::drive(int pwm_speed, bool dir)
{
  // if the new speed value takes place in the opp direction, then reduce the motor speed to zero first
  if (dir != m_cache_dir)
  {
    analogWrite(m_pin_pwm, constrain(0, 0, 255));
  }

  analogWrite(m_pin_pwm, constrain(pwm_speed, 0, 255));
  if (dir)
  {
    digitalWrite(m_pin_in1, HIGH);
    digitalWrite(m_pin_in2, LOW);
  }
  else
  {
    digitalWrite(m_pin_in1, LOW);
    digitalWrite(m_pin_in2, HIGH);
  }

  // store the last state of driver for reference
  m_cache_speed = pwm_speed;
  m_cache_dir = dir;
}

void Driver::stop_driver()
{
  analogWrite(m_pin_pwm, 0);
  digitalWrite(m_pin_in1, LOW);
  digitalWrite(m_pin_in2, LOW);
}

AugerMotor::AugerMotor(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : Driver(pin_pwm, pin_in1, pin_in2) {};

void AugerMotor::init_motor()
{
  init_driver();
}

void AugerMotor::drive_motor(int pwm_speed, bool dir)
{
  drive(pwm_speed, dir);
}

void AugerMotor::stop_motor()
{
  stop_driver();
}

LeadScrewMotor::LeadScrewMotor(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : Driver(pin_pwm, pin_in1, pin_in2) {};

void LeadScrewMotor::init_motor()
{
  init_driver();
}

void LeadScrewMotor::drive_motor(int pwm_speed, bool dir)
{
  drive(pwm_speed, dir);
}

void LeadScrewMotor::stop_motor()
{
  stop_driver();
}

LinearActuator::LinearActuator(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : Driver(pin_pwm, pin_in1, pin_in2)
{
  m_instance_la = this;
};

LinearActuator *LinearActuator::m_instance_la = nullptr;

void LinearActuator::init_motor()
{
  init_driver();
}

void LinearActuator::extend()
{
  drive(m_pwm_speed, true);
  m_timer.begin(isr_timer_router, (m_pwm_dur * 1000 * 1000));
}

void LinearActuator::retract()
{
  drive(m_pwm_speed, false);
  m_timer.begin(isr_timer_router, (m_pwm_dur * 1000 * 1000));
}

void LinearActuator::extend(int pwm_speed, float req_ext)
{
  float pwm_dur = ((req_ext) / (m_oem_max_speed * 1));
  drive(pwm_speed, true);
  m_timer.begin(isr_timer_router, (pwm_dur * 1000 * 1000));
}

void LinearActuator::retract(int pwm_speed, float req_ext)
{
  float pwm_dur = ((req_ext) / (m_oem_max_speed * 1));
  drive(pwm_speed, false);
  m_timer.begin(isr_timer_router, (pwm_dur * 1000 * 1000));
}

void LinearActuator::stop_motor()
{
  stop_driver();
}

void LinearActuator::isr_timer_router()
{
  if (m_instance_la != nullptr)
  {
    m_instance_la->handle_isr();
  }
}

void LinearActuator::handle_isr()
{
  m_timer.end();
  stop_motor();
}

void LinearActuator::home(bool dir)
{
  m_home_dur += 1; // add 1 more second(s) for homing duration
  if (dir)
  {
    drive(m_pwm_speed, false);
    m_timer.begin(isr_timer_router, (m_home_dur * 1000 * 1000));
  }
  else
  {
    drive(m_pwm_speed, true);
    m_timer.begin(isr_timer_router, (m_home_dur * 1000 * 1000));
  }
}

CustomServo::CustomServo(uint8_t servo_pin)
    : m_servo_pin(servo_pin),
      m_current_normalized(0.0f),
      m_target_normalized(0.0f),
      m_last_update_ms(0)
{
}

void CustomServo::init()
{
  m_servo.attach(m_servo_pin, m_min_us, m_max_us);
  m_servo.writeMicroseconds(normalized_to_us(m_current_normalized));
}

void CustomServo::set_target(float target_normalized)
{
  if (target_normalized < 0.0f)
    target_normalized = 0.0f;
  if (target_normalized > 1.0f)
    target_normalized = 1.0f;

  m_target_normalized = target_normalized;
}

void CustomServo::update()
{
  unsigned long now_ms = millis();
  unsigned long dt_ms = now_ms - m_last_update_ms;

  if (dt_ms == 0)
    return;
  m_last_update_ms = now_ms;

  const float max_step = m_slew_deg_per_sec / 270.0f / 1000.0f * (float)dt_ms;
  float diff = m_target_normalized - m_current_normalized;

  if (diff > max_step)
  {
    m_current_normalized += max_step;
  }
  else if (diff < -max_step)
  {
    m_current_normalized -= max_step;
  }
  else
  {
    m_current_normalized = m_target_normalized;
  }

  m_servo.writeMicroseconds(normalized_to_us(m_current_normalized));
}

int CustomServo::normalized_to_us(float t)
{
  return (int)(m_min_us + t * (m_max_us - m_min_us));
}

LimitSwitch *LimitSwitch::m_instances_ls[LIMIT_SWITCH_INSTANCES] = {nullptr};
uint8_t LimitSwitch::m_instance_count_ls = 0;

void LimitSwitch::isr_router_ls_0()
{
  if (m_instances_ls[0])
    m_instances_ls[0]->handle_isr();
}

void LimitSwitch::isr_router_ls_1()
{
  if (m_instances_ls[1])
    m_instances_ls[1]->handle_isr();
}

LimitSwitch::LimitSwitch(uint8_t pin_switch, uint32_t debounce_ms)
    : m_pin_switch(pin_switch),
      m_debounce_ms(debounce_ms),
      m_triggered(false),
      m_last_interrupt_time(0) {};

void LimitSwitch::init()
{
  pinMode(m_pin_switch, INPUT_PULLUP);
  if (m_instance_count_ls < LIMIT_SWITCH_INSTANCES)
  {
    uint8_t id = m_instance_count_ls;
    m_instances_ls[id] = this;
    m_instance_count_ls++;

    void (*isr_func)() = nullptr;
    if (id == 0)
      isr_func = isr_router_ls_0;
    else if (id == 1)
      isr_func = isr_router_ls_1;

    if (isr_func != nullptr)
    {
      attachInterrupt(digitalPinToInterrupt(m_pin_switch), isr_func, CHANGE);
    }
  }
}

void LimitSwitch::handle_isr()
{
  uint32_t current_time = millis();
  // debounce
  if (current_time - m_last_interrupt_time > m_debounce_ms)
  {
    m_triggered = true;
    m_last_interrupt_time = current_time;
  }
}

bool LimitSwitch::is_triggered()
{
  if (m_triggered)
  {
    m_triggered = false;
    return true;
  }
  return false;
}

LoadCell::LoadCell(uint8_t dout_pin, uint8_t sck_pin, float scale_factor)
    : m_dout_pin(dout_pin),
      m_sck_pin(sck_pin),
      m_scale_factor(scale_factor),
      m_current_weight(0.0f),
      m_empty_tare_weight(0.0f), // Initialize it here
      m_lid_tare_weight(0.0f),
      m_is_stable(false),
      m_last_read_ms(0),
      m_buffer_idx(0)
{
  for (uint8_t i = 0; i < STABILITY_BUFFER_SIZE; i++)
  {
    m_recent_reads[i] = 0.0f;
  }
}

void LoadCell::init()
{
  m_load_cell.begin(m_dout_pin, m_sck_pin);
  m_load_cell.set_scale(m_scale_factor);

  // Resets the internal HX711 offset to 0 during boot
  if (m_load_cell.is_ready())
  {
    m_load_cell.tare(10); // Takes average of 10 readings to set a solid baseline[cite: 11]
  }
}

bool LoadCell::is_ready()
{
  return m_load_cell.is_ready();
}

void LoadCell::tare_empty()
{
  // Capture the current EMA weight as the empty container offset
  m_empty_tare_weight = m_current_weight;
  m_lid_tare_weight = 0.0f;
}

void LoadCell::tare_with_lid()
{
  // Set the lid weight relative to the empty container tare
  m_lid_tare_weight = m_current_weight - m_empty_tare_weight;
}

// ... (keep set_scale(), get_scale(), get_raw_value() the same)

float LoadCell::get_soil_weight()
{
  // Net weight = Total - Empty Container - Lid
  float net_weight = m_current_weight - m_empty_tare_weight - m_lid_tare_weight;
  return net_weight; // Clamp noise near zero if needed
}

void LoadCell::set_scale(float scale_factor)
{
  m_scale_factor = scale_factor;
  m_load_cell.set_scale(m_scale_factor);
}

float LoadCell::get_scale()
{
  return m_scale_factor;
}

float LoadCell::get_raw_value(uint8_t samples)
{
  if (m_load_cell.is_ready())
  {
    return m_load_cell.read_average(samples);
  }
  return 0;
}

bool LoadCell::is_stable()
{
  return m_is_stable;
}

void LoadCell::update()
{
  uint32_t now = millis();
  // Read at 10Hz (100ms) to avoid blocking execution
  if (now - m_last_read_ms >= 100)
  {
    m_last_read_ms = now;
    if (m_load_cell.is_ready())
    {
      float raw_weight = m_load_cell.get_units(1);

      // 1. Exponential Moving Average for low-pass filtering
      m_current_weight = (0.25f * raw_weight) + (0.75f * m_current_weight);

      // 2. Ring buffer to verify stability (e.g., lid settling or soil falling)
      m_recent_reads[m_buffer_idx] = m_current_weight;
      m_buffer_idx = (m_buffer_idx + 1) % STABILITY_BUFFER_SIZE;

      float max_val = m_recent_reads[0];
      float min_val = m_recent_reads[0];
      for (uint8_t i = 1; i < STABILITY_BUFFER_SIZE; i++)
      {
        if (m_recent_reads[i] > max_val)
          max_val = m_recent_reads[i];
        if (m_recent_reads[i] < min_val)
          min_val = m_recent_reads[i];
      }

      // If peak-to-peak variation across last 5 reads is < 0.5g, reading is stable
      m_is_stable = ((max_val - min_val) < 0.5f);
    }
  }
}
