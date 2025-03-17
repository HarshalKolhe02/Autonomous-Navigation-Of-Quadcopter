# Autonomous Navigation of Drone:
This is the Repository for the Autonomous Navigation of Quadcopter.

## 🚀 Project Overview
This project focuses on developing an **autonomous aerial vehicle (drone)** capable of navigating a **GPS-denied environment** using **sensor fusion** and **computer vision**. The drone integrates **Intel Depth Camera, IMU, Optical Flow Meter, and LiDAR** for obstacle detection and path planning. It uses **ArduPilot as the flight controller** and a **Jetson Orin Nano** for real-time processing and AI-driven decision-making.

## 🌟 Features
- ✅ **GPS-Denied Navigation** using IMU, Optical Flow, and LiDAR
- ✅ **ArUco Marker Tracking** for waypoint navigation
- ✅ **Autonomous Path Planning** using AI-based algorithms
- ✅ **Safe Spot Detection** via Depth Maps & Computer Vision
- ✅ **ArduPilot-Based Flight Control** using Cube Orange Plus
- ✅ **Failsafe Mechanism** ensuring emergency landing protocols

---

## 🛠️ System Architecture

### 📡 **Hardware Components**
- **Flight Controller**: Cube Orange Plus with:
  - Triple redundant IMUs for fault tolerance.
  - Dual barometers for accurate altitude estimation.
  - Integrated vibration isolation for sensor stability.
- **Companion Computer**: Jetson Orin Nano:
  - 6-core ARM Cortex-A78AE CPU for parallel computing.
  - 1024-core Ampere GPU for real-time depth processing.
  - 8GB LPDDR5 RAM for efficient AI model execution.
- **Sensors**:
  - **Intel RealSense D435i**: Stereo depth camera with onboard IMU for safe spot detection.
  - **TFMini-S LiDAR**: Compact rangefinder for precision altitude measurements.
  - **PX4Flow Optical Flow Sensor**: Provides velocity estimation in GPS-denied scenarios.
  - **IMU & Barometer**: Integrated with Cube Orange for precise stabilization.
- **Frame & Propulsion**:
  - **Frame**: S500 quadcopter frame with carbon fiber structure for durability.
  - **Motors**: AT2312 1150KV motors providing stable thrust and efficiency.
  - **ESCs**: ReadytoSky 40A ESCs for reliable PWM motor control.
  - **Battery**: Orange 4S 5200mAh LiPo, optimized for ~12 minutes of continuous flight.

### 🖥️ **Software Stack**
| Component              | Tools/Frameworks Used |
|------------------------|----------------------|
| **Flight Control**     | ArduPilot, MAVLink, DroneKit |
| **Path Planning**      | Python (A*, RRT, AI-Based Navigation) |
| **Computer Vision**    | OpenCV, PyTorch, Depth Estimation Models |
| **Obstacle Avoidance** | LiDAR Processing, Optical Flow Algorithms |
| **Simulation & Testing** | Gazebo, Ardupilot SITL (Software-in-the-Loop), Mission Planner |

---

## 📜 DroneKit Library Integration

**DroneKit** is a Python library that allows communication between the companion computer and the flight controller (ArduPilot) via the **MAVLink protocol**. It enables the drone to execute autonomous missions by sending flight commands directly from the companion computer.

### **Key Functions of DroneKit in the Project**
- **Autonomous Takeoff & Landing**: The script commands the drone to take off, hover, navigate to waypoints, and land safely.
- **Mission Execution**: DroneKit handles waypoint-based navigation, ArUco marker tracking, and terrain scanning.
- **Failsafe Mechanism**: Monitors telemetry data such as battery voltage, GPS status, and sensor health to trigger emergency landing if required.
- **Real-time Telemetry Processing**: Retrieves flight data (altitude, velocity, position) for monitoring and decision-making.
- **Safe Spot Navigation**: After detecting safe landing zones via depth analysis, DroneKit sends the coordinates as waypoints to ArduPilot for precise landing.

---

## 🔄 Task Execution Flow
1️⃣ **Testing Scripts on ArduPilot SITL**:
   - Simulating drone control in a virtual environment.
   - Testing flight stability, mode transitions, and sensor fusion algorithms.
2️⃣ **Performing Takeoff, Hover, and Landing with GPS Enabled**:
   - Verifying basic flight control parameters.
   - Stabilization testing in real-world conditions.
3️⃣ **Navigating to a Series of ArUco Markers**:
   - Testing waypoint-based navigation.
   - Fine-tuning PID control loops for precision flight.
4️⃣ **Executing the Main Tasks**:
   - **Boundary Identification & Mapping**: Using edge detection to recognize arena limits.
   - **Safe Spot Detection**: Plane segmentation with RANSAC for identifying optimal landing sites.
   - **Autonomous Navigation**: AI-enhanced path planning and real-time terrain adaptation.
   - **Failsafe & Emergency Handling**: Battery monitoring, return-to-home automation, and emergency landing.

---


## Raspberry Pi CubeOrange Integration
To run Dronekit scrpits on Pixhawk, we need to first connect it to Raspberry Pi. This is Done using UART communication protocol. This Pin Map is as follows:   
### PORT:  **TELEM1**
Pinout
| Pin Number | Signal |
|----------|----------|
|1(Red)|VCC|
|2(Black)|TX|
|3(Black)|RX|
|4(Black)|CTS|
|5(Black)|RTS|
|6(Black)|GND|
### Connections
CubeOrange---->RPI   
1. TX---->RX
2. RX---->TX
3. GND---->GND

### Arducopter Parameters
SERIAL1_BAUD=57  
SERIAL1_PROTOCOL=2  
SERIAL5_PROTOCOL=-1  

### Rpi Configuration (One Time)
The Default Serial port of Rpi is used by inbuilt bluetooth Module so in order to use over CubeOrange on Serial communication, We have to disable bluetooth on RPi.   
For this purpose go to rpi boot config file using the following command
```
sudo nano /boot/firmware/config.txt
```
Then add the following line to it 
```
dtoverlay=disable-bt
```
Save the file and exit 

After this we have to make sure UART is enabled on rpi. To do this enter the following command
```
sudo raspi-config
```
After this Follow the Following Sequence
Interface Options->Serial Port->No->Yes

Exit the Configuration menu and reboot RPi

## Running Scripts
**Use connect parameter as**
```
/dev/ttyAMA0
```
and baud as 57600 (Set Earlier) to connect to CubeOrange via Mavlink.

A Sample Script is shown Below:  
```python
from dronekit import connect

vehicle = connect('/dev/ttyAMA0',baud=57600, wait_ready=True)
print("Connected to MavProxy terminal")
```



## Software requirements:
- Ubuntu/ Windows WSL
- Python 3.11
- Maxproxy
- Ardupilot-SITL
- Mission Planner
- OpenCV, DroneKit, Numpy, Time
