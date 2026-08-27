# HSM Aries - Embedded Systems (Drill & Science Modules)

Welcome to the embedded firmware repository for the European Rover Challenge (ERC). This code runs on the Teensy 4.1 microcontroller to control the rover's drill, soil collection, and science mechanisms.

If you are coming from the Arduino IDE and are used to `.ino` files, this repository might look a bit different! We use **PlatformIO** inside **Visual Studio Code (VS Code)**, and we use **micro-ROS** to communicate with the main rover PC (Raspberry Pi).

This guide will explain how the whole system works together.

---

## 1. Where are the `.ino` files? (Understanding PlatformIO)

In the standard Arduino IDE, you write code in a `.ino` sketch, and the software hides a lot of standard C++ structure from you. As our rover software grew complex, we needed a professional build system, which is why we use PlatformIO.

* **Standard C++:** Instead of one big `.ino` file, our code is written in standard C++. The main execution loop is in `src/main.cpp`. Hardware components (like motors and sensors) are split into their own `.cpp` and `.h` (header) files to keep things organized.
* **No Manual Library Installs:** In Arduino, you have to manually download `.zip` files for libraries. In PlatformIO, you don't. Everything the project needs is listed in the `platformio.ini` file. When you open this project, PlatformIO automatically downloads the exact correct versions of the libraries for you.
* **How it Uploads:** You don't need a separate app to upload to the Teensy. When you click the **"Upload" (right arrow) button** at the bottom of VS Code, PlatformIO automatically compiles the code, generates the machine-readable `.hex` file, and pushes it to the Teensy via USB.

---

## 2. What is micro-ROS?

Usually, microcontrollers talk to computers by printing text over `Serial.print()`. For a complex rover, parsing raw text is slow and error-prone.

**micro-ROS** allows our Teensy to act as an actual node on the rover's ROS 2 network. 
* Instead of printing text, the Teensy natively **Subscribes** to ROS 2 topics (like `/motor1/cmd_speed`) and **Publishes** to topics (like `/linact/state`).
* This means the code running on the Raspberry Pi can send exact data types (like `Float32` or `Int32`) directly to the Teensy without any messy text conversion.

---

## 3. The Toolchain: How a command reaches a motor

Here is the exact path of how source code turns into physical movement on the rover:

### Phase A: Writing & Flashing the Code
1. **Source Code:** You write C++ code in `main.cpp` using VS Code.
2. **Compilation:** You click "Build". The GCC compiler translates your human-readable C++ into a `.hex` file (raw machine instructions).
3. **Flashing:** You click "Upload". PlatformIO triggers the Teensy Loader utility in the background, which wipes the Teensy's memory and flashes the new `.hex` file over the USB cable.

### Phase B: Live Rover Operation
Once the Teensy is powered on inside the rover, here is how it talks to the main PC (Raspberry Pi):
1. **The Microcontroller:** The Teensy boots up and waits for a connection.
2. **The Agent:** On the Raspberry Pi, we run a program called the `micro_ros_agent`. Think of this agent as a translator. It listens to the USB/Serial cable connected to the Teensy.
3. **The Network:** When a ROS 2 command is issued on the main network (e.g., "Extend the linear actuator"), the Raspberry Pi passes it to the agent, the agent sends it over the USB cable in a compressed format, and the Teensy receives it in its `linact_cext_cmd_callback`. The Teensy then applies voltage to the motor pins.

---

## 4. How to Get Started

1. Download **Visual Studio Code**.
2. Go to the Extensions tab (left sidebar) and install **PlatformIO IDE**.
3. Clone this repository to your computer.
4. Open the `drill-branch` folder in VS Code. PlatformIO will take a minute or two to automatically install the toolchains and libraries based on the `platformio.ini` file.
5. Plug in the Teensy 4.1 via USB.
6. Look at the blue bottom status bar in VS Code:
   * Click the **Checkmark** (✓) to compile the code and check for errors.
   * Click the **Right Arrow** (→) to compile and upload the code to the Teensy.
