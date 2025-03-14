# Autonomous Navigation of Drone:
This is the Repository for the Autonomous Navigation of Quadcopter.

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
