# Autonomous Navigation of Quadcopter

## Project Overview
This project focuses on developing an **autonomous aerial vehicle (quadcopter)** capable of navigating a **GPS-denied environment** using **sensor fusion**.

## System Architecture

### **Hardware Components**
- **Flight Controller**: **Cube Orange Plus:**
  - Triple redundant IMUs for fault tolerance.
  - Dual barometers for accurate altitude estimation.
  - Integrated vibration isolation for sensor stability.
- **Companion Computer**: Raspberry Pi 4 (8GB):
  - Quad-core ARM Cortex-A72 CPU for general-purpose computing.
  - Broadcom VideoCore VI GPU for basic graphics and limited parallel processing.
  - 8GB LPDDR4 RAM for running lightweight AI models and handling multitasking.
- **Sensors**:
  - **TFMini-S LiDAR**: Compact rangefinder for precision altitude measurements.
  - **PX4Flow Optical Flow Sensor**: Provides velocity estimation in GPS-denied scenarios.
- **Frame & Propulsion**:
  - **Frame**: S500 quadcopter frame with carbon fiber structure for durability.
  - **Motors**: AirGear 2216 920KV motors providing stable thrust and efficiency.
  - **ESCs**: AIR 20A ESCs for reliable PWM motor control.
  - **Battery**: Orange 4S 5200mAh LiPo.
d
### **Software Stack**
| Component              | Tools/Frameworks Used |
|------------------------|----------------------|
| **Flight Control**     | ArduPilot, MAVLink, DroneKit |
| **Path Planning**      | Python|
| **Obstacle Avoidance** | LiDAR Processing, Optical Flow Algorithms |
| **Simulation & Testing** | Ardupilot SITL (Software-in-the-Loop), Mission Planner |

---

## DroneKit Library Integration

**DroneKit** is a Python library that allows communication between the companion computer and the flight controller (ArduPilot) via the **MAVLink protocol**. It enables the drone to execute autonomous missions by sending flight commands directly from the companion computer.

## Raspberry Pi CubeOrange Integration
To run Dronekit scripts on Pixhawk, we need to first connect it to Raspberry Pi. This is done using the UART communication protocol. This Pin Map is as follows:   
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
## Simulations
The simulation shown in this section is done by running Ardupilot SITL in WSL and connecting it with mission Planner and DroneKit script over UDP. 
### Takeoff Hover and Land
In this mission, The Quadcopter is made to reach altitude of 10 meters and hover there for 10 seconds. It then safely lands on the ground.
Script Used: Takeoff_hover_land.py

[Result](https://drive.google.com/file/d/1KDGGHPJMcpwloq_pPW9UvpG8WnaL4c8A/view?usp=sharing)



