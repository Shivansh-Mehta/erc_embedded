// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef SCIENCE_H
#define SCIENCE_H

#include <Arduino.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "Zanshin_BME680.h"

// Install via Arduino Library Manager: "SparkFun SCD4x Arduino Library"
#include "SparkFun_SCD4x_Arduino_Library.h"

constexpr float V_REF = 3.3f;
constexpr float ADC_RES = 1023.0f;

class AnalogSensor
{
protected:
  uint8_t m_pin;
  float read_voltage() const;

public:
  AnalogSensor(uint8_t pin);
  virtual ~AnalogSensor() = default;
  void init();
  virtual float get_value() = 0;
};

class DigitalSensor
{
protected:
  // Tracks whether init() actually succeeded. Reads made before/without a
  // successful init() now return a clear sentinel instead of silently
  // touching an uninitialized sensor object.
  bool m_ready = false;

public:
  virtual ~DigitalSensor() = default;
  virtual bool init() = 0;
  virtual void request_read() {}
  bool is_ready() const { return m_ready; }
};

class pHSensor : public AnalogSensor
{
private:
  float m_calibration_offset;

public:
  pHSensor(uint8_t pin, float offset = 0.0f);
  float get_value() override;
  void set_calibration_offset(float offset);
};

class CapacitiveMoistureSensor : public AnalogSensor
{
public:
  CapacitiveMoistureSensor(uint8_t pin);
  float get_value() override;
};

class TDSSensor : public AnalogSensor
{
public:
  TDSSensor(uint8_t pin);
  float get_value() override;
};

class ORPSensor : public AnalogSensor
{
private:
  float m_calibration_offset;

public:
  ORPSensor(uint8_t pin, float offset = 0.0f);

  float get_value() override;
  void set_calibration_offset(float offset);
};

class DS18B20Sensor : public DigitalSensor
{
private:
  uint8_t m_pin;
  OneWire m_oneWire;
  DallasTemperature m_sensor;

public:
  DS18B20Sensor(uint8_t pin);

  bool init() override;
  void request_read() override;
  float get_value();
};

class BME688Sensor : public DigitalSensor
{
private:
  BME680_Class m_sensor;

public:
  BME688Sensor();

  bool init() override;
  void request_read() override; // triggers a forced-mode conversion, non-blocking
  void get_data(float &temp, float &hum, float &press, float &gas);
};

// Talks over Wire1 (SDA=17, SCL=16 on Teensy 4.1) - a separate bus from the
// BME688 above, which is stuck on default Wire since the Zanshin/Zanduino
// library has no way to target an alternate TwoWire bus.
class SCD41Sensor : public DigitalSensor
{
private:
  SCD4x m_sensor;

public:
  SCD41Sensor();

  bool init() override;
  // No request_read() override: SCD4x's own readMeasurement() is already a
  // non-blocking check-and-fetch (it just polls the data-ready flag), so
  // get_value() calls it directly rather than needing an external
  // trigger/wait split.
  float get_value();
};

#endif
