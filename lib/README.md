# HSM Aries - Science Module Library (`lib/`)

Welcome to the custom hardware library for the European Rover Challenge (ERC) environmental science module. This directory contains the object-oriented C++ classes that abstract the physical sensors of the rover.

By keeping hardware logic in these classes, our `main.cpp` remains clean and focuses solely on the micro-ROS communication layer and sequential state machines.

---

## File Structure

The hardware components are divided into logical modules:

* **`science.h` & `science.cpp**`: Contains the core analog and digital sensor classes required for environmental telemetry.
* **`emg.h` & `emg.cpp**`: Contains classes for emergency management and status indicators (e.g., the stack light).

---

## Hardware Classes Overview

### Analog Sensors

All standard analog sensors inherit from a base `AnalogSensor` class to standardize pin configuration and 10-bit ADC voltage calculations.

* **`AnalogSensor`**: The foundational class that manages `analogRead` math and voltage conversions.
* **`pHSensor` & `ORPSensor**`: Analog implementations that apply specific linear transformations and Op-Amp formulas, featuring on-the-fly calibration offset support.
* **`CapacitiveMoistureSensor` & `TDSSensor**`: Implementations that retrieve raw ADC signals to be mapped later against known calibration fluids or bounds.

### Digital Sensors

All bus-based sensors inherit from a `DigitalSensor` class, which enforces a non-blocking `request_read()` architecture to ensure the micro-ROS executor is never interrupted.

* **`DS18B20Sensor`**: A OneWire soil temperature wrapper configured strictly for asynchronous measuring, preventing standard 750ms CPU delays.
* **`BME688Sensor`**: An I2C implementation that fetches ambient temperature, humidity, pressure, and gas resistance simultaneously via memory references.
* **`SCD41Sensor`**: An I2C CO2 sensor class utilizing delayed bus polling for seamless data extraction.

---

## Dependencies

These classes rely on the following external libraries, which are automatically managed by PlatformIO in the `platformio.ini` file:

* **`Wire.h`**: Built-in standard library for I2C communication (BME688, SCD41).
* **`OneWire.h` & `DallasTemperature.h**`: Required for the DS18B20 1-Wire protocol.
* **`Zanshin_BME680`**: High-precision driver for calculating the BME688 environmental metrics.