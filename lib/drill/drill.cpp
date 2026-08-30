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
  // If reversing direction, actually bring the motor to zero and let that
  // settle before flipping the H-bridge direction pins. Previously this set
  // PWM to 0 and then immediately overwrote it with the new speed on the
  // very next line with no gap, so the "safety" zero never had any effect -
  // the direction pins still saw a full-speed reversal with zero dead-time.
  if (dir != m_cache_dir)
  {
    analogWrite(m_pin_pwm, 0);
    delay(5); // brief settling time before reversing - avoids an instant
              // full-speed direction flip on the H-bridge
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
  // Pad this one homing move by 1 extra second, without permanently
  // growing m_home_dur - it must stay the same on every future call.
  float dur = m_home_dur + 1.0f;
  if (dir)
  {
    drive(m_pwm_speed, false);
    m_timer.begin(isr_timer_router, (dur * 1000 * 1000));
  }
  else
  {
    drive(m_pwm_speed, true);
    m_timer.begin(isr_timer_router, (dur * 1000 * 1000));
  }
}

// GripperServo ======================================================================================================
GripperServo::GripperServo(uint8_t servo_pin, int min_us, int max_us,
                           float slew_deg_per_sec, float travel_deg)
    : m_servo_pin(servo_pin),
      m_current_normalized(0.0f),
      m_target_normalized(0.0f),
      m_last_update_ms(0),
      m_initialized(false),
      m_min_us(min_us),
      m_max_us(max_us),
      m_slew_deg_per_sec(slew_deg_per_sec),
      m_travel_deg(travel_deg)
{
}

void GripperServo::init()
{
  m_servo.attach(m_servo_pin, m_min_us, m_max_us);
  m_servo.writeMicroseconds(normalized_to_us(m_current_normalized));

  // Reset the slew clock here, not just in the constructor. Without this,
  // the first update() after boot sees dt_ms = millis() since power-on
  // (often several seconds), computes a huge max_step, and snaps straight
  // to the first target instead of slewing into it.
  m_last_update_ms = millis();
  m_initialized = true;
}

void GripperServo::set_target(float target_normalized)
{
  m_target_normalized = constrain(target_normalized, 0.0f, 1.0f);
}

void GripperServo::update()
{
  if (!m_initialized)
    return;

  unsigned long now_ms = millis();
  unsigned long dt_ms = now_ms - m_last_update_ms;

  if (dt_ms == 0)
    return;
  m_last_update_ms = now_ms;

  const float max_step = m_slew_deg_per_sec / m_travel_deg / 1000.0f * (float)dt_ms;
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

bool GripperServo::is_at_target() const
{
  return fabsf(m_target_normalized - m_current_normalized) < 0.001f;
}

void GripperServo::stop()
{
  // Freeze in place: cancel any pending move by making "target" equal
  // to wherever we currently are.
  m_target_normalized = m_current_normalized;
}

void GripperServo::detach()
{
  m_servo.detach();
  m_initialized = false;
}

int GripperServo::normalized_to_us(float t) const
{
  return (int)lroundf(m_min_us + t * (m_max_us - m_min_us));
}

// LidServo ==========================================================================================================
LidServo *LidServo::m_instance_lid = nullptr;

LidServo::LidServo(uint8_t servo_pin, int neutral_us, int max_deviation_us)
    : m_servo_pin(servo_pin),
      m_neutral_us(neutral_us),
      m_max_deviation_us(max_deviation_us),
      m_current_speed(0.0f)
{
  m_instance_lid = this;
}

void LidServo::init()
{
  m_servo.attach(m_servo_pin);
  stop(); // command the neutral (stop) pulse immediately so it doesn't spin on boot
}

void LidServo::set_speed(float speed)
{
  m_current_speed = constrain(speed, -1.0f, 1.0f);
  int us = m_neutral_us + (int)lroundf(m_current_speed * m_max_deviation_us);
  m_servo.writeMicroseconds(us);
}

void LidServo::stop()
{
  m_current_speed = 0.0f;
  m_servo.writeMicroseconds(m_neutral_us);
}

void LidServo::rotate_for(float speed, float duration_s)
{
  set_speed(speed);
  m_timer.begin(isr_timer_router, (unsigned int)(duration_s * 1000.0f * 1000.0f));
}

void LidServo::isr_timer_router()
{
  if (m_instance_lid != nullptr)
  {
    m_instance_lid->handle_isr();
  }
}

void LidServo::handle_isr()
{
  m_timer.end();
  stop();
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

bool LimitSwitch::is_pressed() const
{
  // INPUT_PULLUP: pin reads LOW when the switch is actively pressed/closed.
  return digitalRead(m_pin_switch) == LOW;
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

  // Wait for the HX711 to report ready, but bounded - this is what actually
  // caused the earlier boot hang. is_ready() itself is a quick non-blocking
  // pin check, but tare()/read_average() loop internally with no timeout of
  // their own, so if the chip never asserts ready (unpowered, unwired, dead),
  // calling tare() unconditionally can block setup() forever. Give it a
  // fixed window instead, and skip the tare if it never comes up.
  uint32_t start_ms = millis();
  while (!m_load_cell.is_ready() && (millis() - start_ms) < 250)
  {
    // busy-wait briefly; setup() blocking for up to 250ms once at boot is
    // fine, hanging forever is not
  }

  if (m_load_cell.is_ready())
  {
    m_load_cell.tare(10); // Takes average of 10 readings to set a solid baseline
  }
  // else: leave m_current_weight at its default 0.0f; get_soil_weight()
  // will just report 0 until this cell comes online, rather than blocking.
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

Pump::Pump(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : Driver(pin_pwm, pin_in1, pin_in2)
{
  m_instance_la = this;
};

Pump *Pump::m_instance_la = nullptr;

void Pump::init_motor()
{
  init_driver();
}

void Pump::release()
{
  drive(m_pwm_flowrate, true);
  m_timer.begin(isr_timer_router, (m_pwm_dur * 1000 * 1000));
}

void Pump::draw()
{
  drive(m_pwm_flowrate, false);
  m_timer.begin(isr_timer_router, (m_pwm_dur * 1000 * 1000));
}

void Pump::release(int pwm_flowrate, float req_vol)
{
  float pwm_dur = ((req_vol) / (m_oem_max_flowrate * 1));
  drive(pwm_flowrate, true);
  m_timer.begin(isr_timer_router, (pwm_dur * 1000 * 1000));
}

void Pump::draw(int pwm_flowrate, float req_vol)
{
  float pwm_dur = ((req_vol) / (m_oem_max_flowrate * 1));
  drive(pwm_flowrate, false);
  m_timer.begin(isr_timer_router, (pwm_dur * 1000 * 1000));
}

void Pump::stop_motor()
{
  stop_driver();
}

void Pump::isr_timer_router()
{
  if (m_instance_la != nullptr)
  {
    m_instance_la->handle_isr();
  }
}

void Pump::handle_isr()
{
  m_timer.end();
  stop_motor();
}

void Pump::home(bool dir)
{
  m_home_dur = 30;
  drive(m_pwm_flowrate, dir);
  m_timer.begin(isr_timer_router, (m_home_dur * 1000 * 1000));
}
