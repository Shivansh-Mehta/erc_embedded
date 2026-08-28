// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include "science.h"

Pump::Pump(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2)
    : m_pin_pwm(pin_pwm),
      m_pin_in1(pin_in1),
      m_pin_in2(pin_in2) {};

void Pump::init_pump()
{
  analogWriteFrequency(m_pin_pwm, 10000);
  pinMode(m_pin_pwm, OUTPUT);
  pinMode(m_pin_in1, OUTPUT);
  pinMode(m_pin_in2, OUTPUT);
  stop_pump();
}

void Pump::drive(int pwm_speed, bool dir)
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

  // store the last state of Pump for reference
  m_cache_speed = pwm_speed;
  m_cache_dir = dir;
}

void Pump::stop_pump()
{
  analogWrite(m_pin_pwm, 0);
  digitalWrite(m_pin_in1, LOW);
  digitalWrite(m_pin_in2, LOW);
}

AnalogSensor::AnalogSensor(uint8_t pin) : m_pin(pin) {}

void AnalogSensor::init()
{
  pinMode(m_pin, INPUT);
}

float AnalogSensor::read_voltage() const
{
  return analogRead(m_pin) * (V_REF / ADC_RES);
}

pHSensor::pHSensor(uint8_t pin, float offset)
    : AnalogSensor(pin), m_calibration_offset(offset) {}

float pHSensor::get_value()
{
  float voltage = read_voltage();
  return (3.5f * voltage) + m_calibration_offset;
}

void pHSensor::set_calibration_offset(float offset)
{
  m_calibration_offset = offset;
}

CapacitiveMoistureSensor::CapacitiveMoistureSensor(uint8_t pin)
    : AnalogSensor(pin) {}

float CapacitiveMoistureSensor::get_value()
{
  // For capacitive moisture, the raw ADC reading maps inversely to water content.
  // (We use m_pin directly, which is protected in the base class)
  return static_cast<float>(analogRead(m_pin));
}

TDSSensor::TDSSensor(uint8_t pin) : AnalogSensor(pin) {}

float TDSSensor::get_value()
{
  // Currently returns the raw ADC value.
  // We can apply a specific voltage-to-ppm polynomial equation here
  // once we calibrate it with a known TDS solution.
  return static_cast<float>(analogRead(m_pin));
}

ORPSensor::ORPSensor(uint8_t pin, float offset)
    : AnalogSensor(pin), m_calibration_offset(offset) {}

float ORPSensor::get_value()
{
  float voltage = read_voltage();

  // Apply standard ORP formula based on 3.3V reference and 75x Op-Amp gain
  float orp_value = ((30.0f * V_REF * 1000.0f) - (75.0f * voltage * 1000.0f)) / 75.0f;

  return orp_value + m_calibration_offset;
}

void ORPSensor::set_calibration_offset(float offset)
{
  m_calibration_offset = offset;
}

DS18B20Sensor::DS18B20Sensor(uint8_t pin)
    : m_pin(pin), m_oneWire(pin), m_sensor(&m_oneWire) {}

bool DS18B20Sensor::init()
{
  m_sensor.begin();

  // CRITICAL: This prevents the library from freezing the Teensy for 750ms.
  // It allows your Sequential State Machine to trigger a read and walk away.
  m_sensor.setWaitForConversion(false);

  // Check if the sensor is physically wired and responding
  if (m_sensor.getDeviceCount() == 0)
  {
    return false;
  }

  return true;
}

void DS18B20Sensor::request_read()
{
  m_sensor.requestTemperatures();
}

float DS18B20Sensor::get_value()
{
  return m_sensor.getTempCByIndex(0);
}

BME688Sensor::BME688Sensor() {}

bool BME688Sensor::init()
{
  if (!m_sensor.begin(I2C_STANDARD_MODE))
  {
    return false;
  }

  m_sensor.setOversampling(TemperatureSensor, Oversample16);
  m_sensor.setOversampling(HumiditySensor, Oversample16);
  m_sensor.setOversampling(PressureSensor, Oversample16);
  m_sensor.setIIRFilter(IIR4);
  m_sensor.setGas(320, 150); // Heater set to 320°C for 150ms

  return true;
}

void BME688Sensor::get_data(float &temp, float &hum, float &press, float &gas)
{
  int32_t raw_temp, raw_hum, raw_press, raw_gas;

  // fetch raw integer values from the Zanshin library
  m_sensor.getSensorData(raw_temp, raw_hum, raw_press, raw_gas);

  // convert library integer formats to standard float units
  temp = raw_temp / 100.0f;   // Celsius
  hum = raw_hum / 1000.0f;    // % Relative Humidity
  press = raw_press / 100.0f; // hPa
  gas = raw_gas / 100.0f;     // Ohms
}

SCD41Sensor::SCD41Sensor(uint8_t i2c_addr) : m_i2c_addr(i2c_addr) {}

bool SCD41Sensor::init()
{
  Wire.beginTransmission(m_i2c_addr);

  // send standard 0x21 0xB1 command: "start periodic measurement"
  Wire.write(0x21);
  Wire.write(0xB1);

  return (Wire.endTransmission() == 0);
}

void SCD41Sensor::request_read()
{
  Wire.beginTransmission(m_i2c_addr);

  // send standard 0xEC 0x05 command: "read measurement"
  Wire.write(0xEC);
  Wire.write(0x05);

  Wire.endTransmission();
}

float SCD41Sensor::get_value()
{
  // read the 12 bytes prepared by the request_read command
  Wire.requestFrom(m_i2c_addr, static_cast<uint8_t>(12));

  if (Wire.available() >= 12)
  {
    uint8_t data[12];
    for (int i = 0; i < 12; i++)
    {
      data[i] = Wire.read();
    }
    // bitwise shift to combine the high and low bytes of the CO2 reading
    return static_cast<float>((uint16_t)data[0] << 8 | data[1]);
  }

  return 0.0f;
}
