
# CANBus Decoder package
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
## CAN massage send
cansend can0 101#0100000000000000 

## CAN massage receive
candump can0  # Listen CAN0

## Record CAN massges
```
record CAN massges `/can0/recv`: 
```
rosbag record /can0/recv
```
record CAN massges `all`: 
```
rosbag record -a
```
