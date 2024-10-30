# slamtec_c1_techdiffbot
This is ROS2 Differential Drive Robot for Mapping and Localization using SLAM and Slamtec C1 Lidar

# Getting Started

This is userguide on how to start  this robot build. This section will cover the following
* Schematic and Assembly
* ROS2 package installation
* Run Command

## Part 1 : Schematic Pin and Components

In this project, i'm using following components
* Slamtec C1 Lidar x 1
* RPi 4B 4GB x 1
* 2 Encoder Motor 333 RPM, 1:30 Gear x 2
* Arduino Nano x 1
* Motor Driver L298N x 1
* 11V 5500mAH battery

### Pin Assignments

**Motor Driver**
 Arduino Pin | L298N Pin
| -------- | --------
| D10      | IN1 |
| D6       | IN2 |
| D9       | IN3 |
| D5       | IN4 |


**Left Motor Encoder**
 Arduino Pin | Encoder Pin | Encoder Color | 
| -------- | --------      | --------      |
| D2       | IN1           | Green         |
| D3       | IN2           | Yellow        |


