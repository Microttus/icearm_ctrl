# IceCube Robot Arm Control

**This is a open source project for control of a 3-axis 
robotic arm using micro ROS, with contol of tooltip based
on inverse kinematics and path planing**

*Note: This only works for servo driven arms*

## How to use

1. Install a Arduino Compiler (ex. [Arduino IDE](www.arduino.cc/Software)) 
2. Upload the code available in the tools folder
3. Launch a micro ros agent on the receiver computer (snap pkg available micro-ros-agent)
4. Download project to ubuntu machine and build
5. Launch the application
6. Post new coordinated to the /icearm_input topic

## Additional information

The project consist of a c++ library with basic functions, 
and a facade class for ROS compatibility. 
The library can be used separately for control of other platforms are 
to be used. 

