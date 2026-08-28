// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef SCIENCE_H
#define SCIENCE_H

#include <Arduino.h>

#include <Wire.h>
#include <OneWire.h>
// #include <DallasTemperature.h>

class PumpController
{
public:
  PumpController(uint8_t pin) : pin_pump_(pin), is_running_(false) {}

  void init()
  {
    pinMode(pin_pump_, OUTPUT);
    stop();
  }

  void start()
  {
    digitalWrite(pin_pump_, HIGH);
    is_running_ = true;
  }

  void stop()
  {
    digitalWrite(pin_pump_, LOW);
    is_running_ = false;
  }

  bool is_running() const
  {
    return is_running_;
  }

private:
  uint8_t pin_pump_;
  bool is_running_;
};

class BaseAnalogSensor
{
public:
  BaseAnalogSensor(uint8_t pin) : pin_(pin) {}

  void init()
  {
    pinMode(pin_, INPUT);
  }

protected:
  uint8_t pin_;
  const float ADC_REF_VOLTAGE = 3.3f;
  const float ADC_MAX_COUNT = 1023.0f;

  // Averages raw ADC values to filter out motor/electrical noise
  float read_raw_averaged(uint8_t samples = 10)
  {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
      sum += analogRead(pin_);
      delayMicroseconds(100); // 100us delay does not block ROS
    }
    return static_cast<float>(sum) / samples;
  }

  // Converts the averaged raw value into a physical voltage
  float read_voltage_averaged(uint8_t samples = 10)
  {
    return read_raw_averaged(samples) * (ADC_REF_VOLTAGE / ADC_MAX_COUNT);
  }
};

class PHSensor : public BaseAnalogSensor
{
public:
  PHSensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  float get_ph()
  {
    float voltage = read_voltage_averaged();
    return 3.5f * voltage; // Replace with actual calibration curve
  }
};

class ORPSensor : public BaseAnalogSensor
{
public:
  ORPSensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  float get_orp_mv()
  {
    float voltage = read_voltage_averaged();
    return (voltage - 1.65f) * 1000.0f; // Offset to 3.3V logic midpoint
  }
};

class TurbiditySensor : public BaseAnalogSensor
{
public:
  TurbiditySensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  float get_turbidity_ntu()
  {
    float voltage = read_voltage_averaged();
    float ntu = -1120.4f * (voltage * voltage) + 5742.3f * voltage - 4353.8f;
    return (ntu < 0) ? 0.0f : ntu;
  }
};

class MoistureSensor : public BaseAnalogSensor
{
public:
  MoistureSensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  float get_moisture_pct()
  {
    float raw = read_raw_averaged();
    float pct = map(raw, 800, 400, 0, 100); // Calibrate dry/wet limits here
    return constrain(pct, 0.0f, 100.0f);
  }
};

class ECSensor : public BaseAnalogSensor
{
public:
  ECSensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  // EC requires temperature compensation, so we pass the temp directly into the read function
  float get_ec(float current_temp_c)
  {
    float voltage = read_voltage_averaged();
    float comp_voltage = voltage / (1.0f + 0.02f * (current_temp_c - 25.0f));
    return (133.42f * pow(comp_voltage, 3) - 255.86f * pow(comp_voltage, 2) + 857.39f * comp_voltage) * 0.5f;
  }
};

class AnalogGasSensor : public BaseAnalogSensor
{
public:
  AnalogGasSensor(uint8_t pin) : BaseAnalogSensor(pin) {}

  float get_raw_gas_level()
  {
    return read_raw_averaged();
  }
};

// class SoilTempSensor
// {
// public:
//   SoilTempSensor(uint8_t pin) : oneWire_(pin), sensor_(&oneWire_) {}

//   void init()
//   {
//     sensor_.begin();
//     sensor_.setWaitForConversion(false); // CRITICAL: Prevents 750ms blocking
//     sensor_.requestTemperatures();
//   }

//   float get_temperature_c()
//   {
//     float temp = sensor_.getTempCByIndex(0);
//     // Kick off the next conversion instantly so it's ready next time
//     sensor_.requestTemperatures();
//     return (temp == DEVICE_DISCONNECTED_C) ? 0.0f : temp;
//   }

// private:
//   OneWire oneWire_;
//   DallasTemperature sensor_;
// };

class BME688Sensor
{
public:
  void init()
  {
    // Wire.begin() usually handled in main.cpp, init sensor here
  }

  void update()
  {
    // Check if I2C data is ready non-blockingly
  }

  float get_air_temp() { return 22.5f; }
  float get_voc_index() { return 100.0f; }
};

class SCD41Sensor
{
public:
  void init() {}
  void update() {}
  uint16_t get_co2_ppm() { return 420; }
};

#endif