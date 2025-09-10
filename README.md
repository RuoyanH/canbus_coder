
# CANBus Coder package
## Features
Decode and encode CAN massges from vehicles based on DBC files
## Install Dependencies
```
sudo apt install can-utils                           #SocketCAN userspace utilities and tools 
sudo apt install ros-$ROS_DISTRO-socketcan-bridge    #ROS Driver 
python3 -m pip install cantools                      #CAN BUS tools in Python 3.
``` 
## Setup Socket CAN
### 1.  **Loader kernel**
Use the `ORIN`  native CAN channel: 
```
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```
OR use `vcan`
```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
```
OR use `USB-CAN`
```
sudo modprobe gs_usb
```
### 2.  **Config Socket CAN**
```
sudo ip link set can0 up type can bitrate 250000
```
## Start Nodes (can_decoder)
Launch the `can_decoder`  node: 
```
roslaunch can_decoder can_decoder.launch
```

## CAN Test Command
```
## listening CAN
cancump can0

## [DigiStop] Step 1: Send 0xF1 (negotiation response) - Machina CxD Data (PGN FACB)
cansend can0 18FACBF2#F100000000000001

## [DigiStop] Step 2: Send 0xF0 (negotiation complete) - Machina CxD Data (PGN FACB)
cansend can0 18FACBF2#F000000000000002

## [DigiStop] Send TimeRequest - Machina CxD Time Request
cansend can0 00EA2AF2#0000000000000000 
```
## Record CAN massges
```
record CAN massges `/can0/recv`: 
record CAN massges `all`: 
```
```
rosbag record /can0/recv
rosbag record -a
```
