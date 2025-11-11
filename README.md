# Gait Control Firmware for Humanoid Robot

## 🧠 Project Overview

This repository contains the **low-level gait control firmware** for the humanoid robot **"ROBINION"**.  
Its primary function is to manage motor initialization, perform Inverse Kinematics (IK) calculations, and generate stable walking gaits.

This firmware serves as the foundation for a larger system that integrates **machine vision (YOLO v8)** for environment perception and object detection.  
The control program focuses on the **execution layer**, handling robot locomotion by receiving target position coordinates through a serial port.

---

## ⚙️ Core Features

### 🔩 Dynamixel Servo Motor Control
- Utilizes the `DynamixelWorkbench` library to initialize Dynamixel motors, manage **Torque On/Off** states, and perform **Sync Write** operations.
- Supports up to **12 actuators** simultaneously, covering leg and head joints.

### 🧮 Inverse Kinematics (IK)
- Implements an IK algorithm tailored for the robot’s **Parallel Kinematics** leg structure.
- Converts end-effector Cartesian coordinates ($x, y, z$) into the required joint angles (`theta_hp`, `theta_ap`, `theta_ar`, `theta_hr`) for each leg.

### 🚶 Gait Generation and Control
- Achieves walking motion using traditional **trajectory planning** that combines IK with **sine/cosine interpolation**.
- Implements a **two-step cyclic walking logic** managed by the `walking()` function:
  - Handles left/right leg exchange
  - Allocates step length
  - Performs half-step adjustments for smooth starting and stopping
  
---

## 🦿 System & Equipment

This firmware operates as the **low-level controller** of the humanoid robot **ROBINION**.

### 🤖 Robot Platform Specifications (ROBINION)

| Item | Specification | 
| :--- | :--- |
| **Height** | 85 cm |
| **Weight** | 7 kg |
| **Degrees of Freedom (DOF)** | 14 (10 legs, 2 arms, 2 head) |
| **Actuators** | MX-106 ×10, MX-64 ×2, AX-12A ×2 |
| **Leg Kinematics** | Parallel Kinematics |
| **Movement Type** | Forward and Inverse Kinematics |
| **System** | Ubuntu 22.04 |
| **Achievement** | 🥇 1st Place – 2025 Taiwan Humanoid Robot Hurocup Marathon (Adult Group) |

---

## 🧰 Hardware Components
### ⚙️ Actuators
- Dynamixel MX-106 ×10
- Dynamixel AX-12A ×2

### 🧠 Controllers
- OpenCM9.04 (Sub-controller)
- ODROID-H2+ (Main controller, Intel J4115)
- Coral USB Accelerator

### 🔋 Power System
- Li-Po Battery 14.8V, 4400mA
- Power Adapter

### 📡 Communication & Accessories
- Wireless Network Card
- Virtual Display Adapter
- Logitech C930e (Camera)

### 📸 Equipment Overview

Below is a summary of the hardware components used in the ROBINION humanoid robot system.

| Equipment | Description | Image |
| :--------- | :----------- | :---- |
| **Dynamixel MX-106** | Main actuator for leg joints | ![MX-106](./images/mx106.jpg) |
| **Dynamixel AX-12A** | Actuator for upper body joints | ![AX-12A](./images/ax12a.jpg) |
| **OpenCM9.04** | Sub-controller for low-level motor control | ![OpenCM9.04](./images/opencm.jpg) |
| **ODROID-H2+** | Main controller (Intel J4115) | ![ODROID-H2+](./images/odroid.jpg) |
| **Coral USB Accelerator** | Edge TPU for AI inference | ![Coral Accelerator](./images/coral.jpg) |
| **Wireless Network Card** | For communication with high-level system | ![Wi-Fi Card](./images/wifi.jpg) |
| **Virtual Display Adapter** | For headless mode operation | ![Virtual Display](./images/display.jpg) |
| **Logitech C930e** | Camera for vision and object detection | ![Logitech C930e](./images/c930e.jpg) |
| **Li-Po Battery** | 14.8V, 4400mA power supply | ![Battery](./images/battery.jpg) |
| **Power Adapter** | External DC adapter for bench testing | ![Adapter](./images/adapter.jpg) |

---

## 💻 Software Environment

| Component | Description | Source |
| :--- | :--- | :--- |
| **Operating System** | Ubuntu 22.04 | |
| **Firmware Development** | Arduino IDE (C++) | |
| **Control Library** | DynamixelWorkbench | *(Used in code)* |
| **High-Level Vision/Control** | Python, PyQt5, OpenCV, PySerial, ROS2 | |
| **Machine Vision Model** | YOLO v8 (Object Detection) | *(From project report)* |

---

## 📁 Code Structure

| File | Description |
| :--- | :--- |
| `[PROJECT_NAME].ino` | Main sketch file. Includes `setup()` and `loop()` functions for initialization and serial command handling. |
| `robotController.h` | Header file. Defines structures (`IKAngles`, `Position`), constants, actuator IDs, and function prototypes. |
| `robotController.cpp` | Implementation file. Contains hardware constants, IK functions (`calculateInverseKinematics`), conversion logic, motor setup (`initializePosition`), movement (`moveToPos`), and gait logic (`walking`). |

---

### Compilation & Upload

1. Install the **DynamixelWorkbench** library in the **Arduino IDE**.  
2. Place `[PROJECT_NAME].ino`, `robotController.h`, and `robotController.cpp` in the same folder.  
3. Select the correct board (**OpenCM9.04** or **OpenCR**) and serial port.  
4. Compile and upload the firmware to the controller.

---

## 🏆 Acknowledgment

This firmware is part of the **ROBINION Humanoid Project**, developed under the guidance of **Prof. Jaesik Jeong**  
and the **AI & Robotics Research Team**.  
Special thanks to all contributors for their dedication to motion control and robotic gait optimization.

---
