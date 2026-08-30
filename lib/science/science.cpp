// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include "science.h"

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
  float slope = ((7.0 - 4.0) / (1.5 - 2.03));
  float raw_ph = 7.0 + (slope * (voltage - 1.5));
  return raw_ph + m_calibration_offset; // FIX: offset was computed but never applied before
}

void pHSensor::set_calibration_offset(float offset)
{
  m_calibration_offset = offset;
}

CapacitiveMoistureSensor::CapacitiveMoistureSensor(uint8_t pin)
    : AnalogSensor(pin) {}

float CapacitiveMoistureSensor::get_value()
{
  float raw = static_cast<float>(analogRead(m_pin));
  // Air = 759.0, Water = 403.0
  float moisture_pct = ((759.0f - raw) / (759.0f - 403.0f)) * 100.0f;
  if (moisture_pct < 0.0f)
    return 0.0f;
  if (moisture_pct > 100.0f)
    return 100.0f;
  return moisture_pct;
}

TDSSensor::TDSSensor(uint8_t pin) : AnalogSensor(pin) {}

float TDSSensor::get_value()
{
  // 1. Get the actual voltage (using your base class function)
  float voltage = read_voltage();

  // 2. Apply a calibration factor.
  // Still a placeholder - calculate this by placing the sensor in a known TDS fluid.
  // Example: If a 1000 ppm fluid gives a reading of 1.5V, your factor is (1000 / 1.5) = 666.67
  float calibration_factor = (194.107 / 0.713); // Replace with your tested number

  float tds_value = voltage * calibration_factor;

  // Optional: clamp to 0 if noise pulls the voltage slightly negative
  return (tds_value < 0.0f) ? 0.0f : tds_value;
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
  pinMode(m_pin, INPUT_PULLUP);
  m_sensor.begin();

  // CRITICAL: This prevents the library from freezing the Teensy for 750ms.
  // It allows your Sequential State Machine to trigger a read and walk away.
  m_sensor.setWaitForConversion(false);
  m_sensor.setResolution(12); // explicit - matches the 750ms conversion-time assumption exactly

  // Check if the sensor is physically wired and responding
  m_ready = (m_sensor.getDeviceCount() != 0);
  return m_ready;
}

void DS18B20Sensor::request_read()
{
  if (!m_ready)
    return; // don't trigger a conversion on a probe that was never found
  m_sensor.requestTemperatures();
}

float DS18B20Sensor::get_value()
{
  if (!m_ready)
    return -127.0f; // matches the DallasTemperature library's own "disconnected" sentinel

  return m_sensor.getTempCByIndex(0);
}

BME688Sensor::BME688Sensor() {}

bool BME688Sensor::init()
{
  m_ready = m_sensor.begin(I2C_STANDARD_MODE);
  if (!m_ready)
    return false;

  m_sensor.setOversampling(TemperatureSensor, Oversample16);
  m_sensor.setOversampling(HumiditySensor, Oversample16);
  m_sensor.setOversampling(PressureSensor, Oversample16);
  m_sensor.setIIRFilter(IIR4);
  m_sensor.setGas(320, 150); // Heater set to 320 degrees C for 150ms

  return true;
}

void BME688Sensor::request_read()
{
  if (!m_ready)
    return;

  // Non-blocking: just starts a forced-mode conversion on the sensor.
  // The result is collected later in get_data() once the caller's tick
  // counter has waited out the conversion + gas-heater soak time.
  m_sensor.triggerMeasurement();
}

void BME688Sensor::get_data(float &temp, float &hum, float &press, float &gas)
{
  if (!m_ready)
  {
    temp = hum = press = gas = NAN; // makes a failed/never-initialized read visible downstream
    return;
  }

  int32_t raw_temp, raw_hum, raw_press, raw_gas;

  // waitSwitch = false: we've already waited out the conversion time via the
  // caller's tick counter (see request_read()), so this just grabs the
  // latest completed reading instead of blocking the ROS executor here.
  m_sensor.getSensorData(raw_temp, raw_hum, raw_press, raw_gas, false);

  // convert library integer formats to standard float units
  temp = raw_temp / 100.0f;   // Celsius
  hum = raw_hum / 1000.0f;    // % Relative Humidity
  press = raw_press / 100.0f; // hPa
  gas = raw_gas / 100.0f;     // Ohms
}

SCD41Sensor::SCD41Sensor() {}

bool SCD41Sensor::init()
{
  // .begin() already calls stopPeriodicMeasurement() internally (500ms
  // blocking - acceptable here since init() is a one-time bring-up call,
  // never on the fast telemetry read path) before verifying presence via
  // getSerialNumber(). That's exactly the fix needed for a sensor left
  // running periodic measurement from a previous session - see the
  // earlier hand-rolled version's comments for why that mattered.
  m_ready = m_sensor.begin(Wire1);
  if (!m_ready)
    return false;

  m_ready = m_sensor.startPeriodicMeasurement();
  return m_ready;
}

float SCD41Sensor::get_value()
{
  if (!m_ready)
    return -1.0f;

  // readMeasurement() is itself a non-blocking check-and-fetch: it polls the
  // data-ready flag and only pulls a new sample over I2C if one's actually
  // waiting, returning false immediately otherwise (periodic mode only
  // produces a new sample every ~5s).
  if (!m_sensor.readMeasurement())
    return -1.0f;

  return static_cast<float>(m_sensor.getCO2());
}
