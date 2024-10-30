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
 Arduino Pin | Encoder Pin | Encoder Color | Motor Connection         |
| -------- | --------      | --------      | --------                 |
|          | M1 (Motor)    | Red           | Heatsink Left (Bottom)   |
| GND      | GND           | Black         |                          |
| D3(Intr) | C1            | Yellow        |                          |
| D2(Intr) | C2            | Green         |                          |
| 5V       | VCC(Encoder)  | Blue          |                          |
|          | M2 (Motor)    | White         | Heatsink Left (Top)      |


**Right Motor Encoder**
 Arduino Pin | Encoder Pin | Encoder Color | Motor Connection        |
| -------- | --------      | --------      | --------                |
|          | M1 (Motor)    | Red           | Heatsink Right (Bottom) |
| GND      | GND           | Black         |                         |
| A4(Ana)  | C1            | Yellow        |                         |
| A5(Ana)  | C2            | Green         |                         |
| 5V       | VCC(Encoder)  | Blue          |                         |
|          | M2 (Motor)    | White         | Heatsink Right (Top)    |


## Part 2 : Package Installation Inside RPI and PC

***For, RPI*** i'm using the following specifications and flash the SD Card using RaspberryPi Imager
```
Ubuntu 64-bit
Ubuntu 22.04.5 LTS
Desktop Version
```

Please make sure you have install ROS2 Humbles inside your `RPI` and the associated packages. Please refer to this documents
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ros2_humble_installation_packages_guide.md
```


