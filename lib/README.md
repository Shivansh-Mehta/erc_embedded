# HSM Aries - Hardware Abstraction Library (`lib/`)

Welcome to the custom hardware library for the European Rover Challenge (ERC) firmware. This directory contains the object-oriented C++ classes that abstract the physical hardware components of the rover. 

By keeping hardware logic in these classes, our `main.cpp` remains clean and focuses solely on the micro-ROS communication layer.

---

## File Structure

The hardware components are divided into logical modules:
*   **`drill.h` & `drill.cpp`**: Contains the core actuator, motor, and sensor classes for the drilling and soil collection mechanisms.
*   **`emg.h` & `emg.cpp`**: Contains classes for emergency management and status indicators (e.g., the stack light).

---

## Hardware Classes Overview

### Motors & Actuators
All heavy-duty motors inherit from a base `Driver` class to standardize PWM and direction control.
*   **`Driver`**: The foundational class that manages PWM frequency (`analogWriteFrequency`), speed constraints, and dual-pin direction configuration. 
*   **`AugerMotor`**: Inherits from `Driver` to control the rotational drill mechanism.
*   **`LeadScrewMotor`**: Inherits from `Driver` to control the vertical translation of the drill.
*   **`LinearActuator`**: Inherits from `Driver` but adds complex, time-based position tracking. It utilizes the Teensy's hardware `IntervalTimer` to execute non-blocking extensions, retractions, and homing routines based on the OEM speed constraints (15.0 mm/s).
*   **`GripperServo`**: A custom wrapper around the standard `Servo.h` library. It includes a built-in slew-rate limiter (`update()`) to ensure the servo moves smoothly to its target rather than snapping violently, which protects the physical hardware.

### Sensors & Inputs
*   **`LimitSwitch`**: An interrupt-driven hardware switch handler. It automatically configures the pins as `INPUT_PULLUP` and attaches a `FALLING` interrupt. It features a built-in software debounce mechanism to prevent false triggers.
*   **`LoadCell`**: A dedicated class utilizing the `HX711` library to interface with the strain gauges for weighing soil samples.

### Indicators
*   **`StackLight`**: Controls the three-color (Green, Yellow, Red) visual indicator on the rover. It provides a simple `state()` method to instantly switch between predefined color configurations.

---

## Dependencies

These classes rely on the following external libraries, which are automatically managed by PlatformIO in the `platformio.ini` file:
*   **`Servo.h`**: For standard PWM servo control.
*   **`HX711.h`**: For reading the load cell amplifiers.
*   **`IntervalTimer.h`**: Teensy core library for hardware-level timer interrupts.