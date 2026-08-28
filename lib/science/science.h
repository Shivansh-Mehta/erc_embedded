// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef SCIENCE_H
#define SCIENCE_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "Zanshin_BME680.h"

constexpr float V_REF = 3.3f;
constexpr float ADC_RES = 1023.0f;

class Pump
{
public:
  Pump(uint8_t pin_pwm, uint8_t pin_in1, uint8_t pin_in2);
  void init_pump();
  void drive(int pwm_speed, bool dir);
  void stop_pump();

private:
  uint8_t m_pin_pwm;
  uint8_t m_pin_in1;
  uint8_t m_pin_in2;
  int m_cache_speed;
  bool m_cache_dir;
};

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
public:
  virtual ~DigitalSensor() = default;
  virtual bool init() = 0;
  virtual void request_read() {}
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
  void get_data(float &temp, float &hum, float &press, float &gas);
};

class SCD41Sensor : public DigitalSensor
{
private:
  uint8_t m_i2c_addr;

public:
  SCD41Sensor(uint8_t i2c_addr = 0x62);

  bool init() override;
  void request_read() override;
  float get_value();
};

#endif
