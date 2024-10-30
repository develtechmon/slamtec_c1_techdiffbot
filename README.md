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


**Motor Specifications**
 Params    | Value
| -------- | --------
| Gear     | 1:30           |
| RPM      | 333            |
| CPR      | 11x4x30 = 1320 |
| PPR      | 11 x4 = 44     |


## Part 2 : Package Installation Inside RPI and PC

This is my components for this tutorial
* SD Card 64 GB
* Raspberry Pi Imager
* RPI 4 4GB
* HDMI and Keyboard
  
For, `RPI` i'm using the following image which is selected from RPI imageand flash the SD Card using this same tool
```
Ubuntu 64-bit
Ubuntu 22.04.5 LTS
Desktop Version
```

Once done, please connect your `Flashed SD cards` to RPI 4B and follow the setup from there to connect to network and do necessary installation.

In next step, you're going to install ROS2 Humbles inside your `RPI` and `PC` and all  associated packages. Please refer to this documents on how to do this setup.
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ros2_humble_installation_packages_guide.md
```

## Part 3 : Enable SSH and Disable Graphic Inside RPI

For most of time, you will need to work and connect with `SSH` to RPI and work only with console instead of graphic interface. 

### Enable `SSH`
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ssh_installation.md
```

### Disable Graphic To Save `RAM`
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ros2_disable_enable_rpi_graphic.md
```

## Part 4 : 
