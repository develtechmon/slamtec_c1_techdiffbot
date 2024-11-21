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
You can find and check this information inside `motor_driver.h` which is inside `ROSArduinoBridge` package
 Arduino Pin | L298N Pin | Arduino Variable     | 
| -------- | --------    | --------             |
| D10      | IN1         | LEFT_MOTOR_FORWARD   |
| D6       | IN2         | LEFT_MOTOR_BACKWARD  |
| D9       | IN3         | RIGHT_MOTOR_FORWARD  | 
| D5       | IN4         | RIGHT_MOTOR_BACKWARD |

**Left Motor Encoder**
You can find and check this information inside `encoder_driver.h` which is inside `ROSArduinoBridge` package
 Arduino Pin | Encoder Pin | Encoder Color | Motor Connection         |  Arduino Variable  |
| -------- | --------      | --------      | --------                 |  --------          |  
|          | M1 (Motor)    | Red           | Heatsink Left (Bottom)   |                    |
| GND      | GND           | Black         |                          |                    |
| D3(Intr) | C1            | Yellow        |                          | LEFT_ENC_PIN_A PD2 |
| D2(Intr) | C2            | Green         |                          | LEFT_ENC_PINB PD3  |
| 5V       | VCC(Encoder)  | Blue          |                          |                    |
|          | M2 (Motor)    | White         | Heatsink Left (Top)      |                    |


**Right Motor Encoder**
You can find and check this information inside `encoder_driver.h` which is inside `ROSArduinoBrdige` package
 Arduino Pin | Encoder Pin | Encoder Color | Motor Connection        | Arduino Variable    |
| -------- | --------      | --------      | --------                | --------            |
|          | M1 (Motor)    | Red           | Heatsink Right (Bottom) |                     |
| GND      | GND           | Black         |                         |                     |
| A4(Ana)  | C1            | Yellow        |                         | RIGHT_ENC_PIN_A PC4 |
| A5(Ana)  | C2            | Green         |                         | RIGHT_ENC_PIN_B PC5 |
| 5V       | VCC(Encoder)  | Blue          |                         |                     |
|          | M2 (Motor)    | White         | Heatsink Right (Top)    |                     |

**Motor Specifications**
 Params    | Value
| -------- | --------
| Gear     | 1:30           |
| RPM      | 333            |
| CPR      | 11x4x30 = 1320 |
| PPR      | 11 x4 = 44     |

This is good website that explain the details parameters of motor we used in this project. 
It shows critical parameters such as `CPR`, `PPR`. Please visit this page to understand the details of our motor
```
https://category.yahboom.net/products/md520?variant=39910808911956
```

## Upload Arduino Code
And lastly, you will have to upload `ROSArduinoBridge.ino` code into your `Arduino Nano`. Please refer to this
userguide on how to `install` and `test` if Encoder and Motor is working. This code is already compiled together in this package.
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ros2_control_arduino_setup.md
```

## Part 2 : Package Installation Inside RPI and PC

This is my components for this tutorial
* SD Card 64 GB
* Raspberry Pi Imager
* RPI 4 4GB
* HDMI and Keyboard
  
For, `RPI` i'm using the following image which is selected from RPI imager and flash the SD Card using this same tool
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

## Part 4 : Install Slamtec C1 package in RPI and PC

To install this package, please refer to this userguide
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/ros2_slamtec_rplidar_installation.md
```

## Part 5 : Configure `STEAM Deck` Controller

Please refer to this userguide on how to setup our `Steam Deck` controller which will be used later to control the robot
```
https://github.com/develtechmon/ROS2/blob/main/UserGuide/Steam_Controller_Joystick.md
```

## Part 6 : Git Clone this package

Now let's start building and compiler our robot. Open new terminal in PC or SSH to RPI and please do the following:
```
git clone https://github.com/develtechmon/slamtec_c1_techdiffbot.git -b with-mpu6050
cd slamtec_c1_techdiffbot/
cd bumperbot_ws/

rosdep install --from-paths src --ignore-src -r -y
rosdep update && rosdep install --from-path src --ignore-src -y
colcon build
```

If you're running in `RPi Zero 2W`, you can consider to run build command as follow. To manage
RAM usage
```
colcon build --parallel-workers 2
```

## Install IMU Package 

This is connection IMU to Raspberry Pi

**MPU6050*
 Sensor    | Rpi Pin
| -------- | --------
| Vcc      | VCC   |
| Gnd      | GND   |
| SCL      | SCL   |
| SDA      | SDA   |


This is additional userguide on how to install `MPU6050` in this robot. In your RPI or PC, please install this package first
```
sudo apt-get install python3-smbus
pip3 install smbus
```
When you run this command,
```
sudo i2cdetect -y 1
```
you will see your `I2C` address is detected as follow. This show our MPU6050 is recognized by `RaspberryPi`
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --   
```

## Launch Our Robot and SLAM !

## Simulation 
In your PC, please go to  following command
```
cd ~/Desktop/My_Project/slamtec_c1_techdiffbot/bumperbot_ws
```
We need to enable this setting to start `mapping` as follow
```
vi src/bumperbot_bringup/config/mapper_params_online_async.yaml 
```
Then enable this file as follow:
```
# ROS Parameters
   odom_frame: odom
   map_frame: map
   base_frame: base_footprint
   scan_topic: /scan
   use_map_saver: true
   
   mode: mapping <----------- Enable this
```
Run this command to launch our `SIMULATION` robot in PC
```
ros2 launch bumperbot_bringup gazebo.sim.launch.py world:=./src/bumperbot_bringup/world/map_v1 
```

Once done, then we can run `SLAM` either from PC and set `use_sim_time:=True`
```
ros2 launch bumperbot_bringup slam.launch.py use_sim_time:=True
```

Launch `rviz`
```
rviz2 -d src/bumperbot_bringup/rviz2/slam_rviz.rviz
```

Once done with mapping, return to its original position and please save the map and our Gazebo world as well and please ensure the robot 

## Real Robot 
In your RPI, please go to  following command
```
cd ~/Desktop/My_Project/slamtec_c1_techdiffbot/bumperbot_ws
```

Run this command to launch our REAL robot in RPI
```
ros2 launch bumperbot_bringup real_robot.launch.py 
```

Once done, then we can run `SLAM` either from RPI or PC and set `use_sim_time:=False`
```
ros2 launch bumperbot_bringup slam.launch.py use_sim_time:=False
```

Launch `rviz`
```
rviz2 -d src/bumperbot_bringup/rviz2/slam_rviz.rviz
```

Once done with mapping, return to its original position and please save the map and our real world as well and please ensure the robot 

## Move and Control Your robot

You can use your `Steam Controller` Joystick or any other Joystick to control the robot. When you launch this script, 
it automatically recognize our `joystick`. 

However, if you don't have joystick, we can control the robot using keyboard. Please use this command as follow:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

## Launch our Robot and Nav2 !

## Simulation
In your PC, please go to  following command
```
cd ~/Desktop/My_Project/slamtec_c1_techdiffbot/bumperbot_ws
```

We need to enable this setting to start `localization` as follow
```
vi src/bumperbot_bringup/config/mapper_params_online_async.yaml 
```

Then enable this file as follow:
```
# ROS Parameters
   odom_frame: odom
   map_frame: map
   base_frame: base_footprint
   scan_topic: /scan
   use_map_saver: true
   
   mode: localization <--------- Enable this
   map_file_name: /home/jlukas/Desktop/My_Project/slamtec_c1_techdiffbot/bumperbot_ws/src/bumperbot_bringup/map/serialize_map_v1 <--------- Set this path to our saved map
   map_start_at_dock: true <--------- Enable this
```
Run this command to launch our `SIMULATION` robot in PC
```
ros2 launch bumperbot_bringup gazebo.sim.launch.py world:=src/bumperbot_bringup/world/map_v1
```

Once done, then we can run `SLAM` either from PC and set `use_sim_time:=True`
```
ros2 launch bumperbot_bringup slam.launch.py use_sim_time:=True
```

Launch our `NAV2`
```
ros2 launch bumperbot_bringup navigation_launch.py use_sim_time:=true
```

Launch `rviz`
```
rviz2 -d src/bumperbot_bringup/rviz2/slam_rviz.rviz
```

and select the `rviz` configuration as follow
```
Fixed Frame - map
Map Topic - Global../Costmap
Color Scheme - Costmap
Path - Topic - /plan
```

From `rviz`
```
Select 2D Goal Pose and start navigating
```

## Launch our Robot using AMCL and Nav2 !

This is the modifications i made that combine both `localization` and `navigation` launch file.

## Simulation
In your PC, please go to  following command
```
cd ~/Desktop/My_Project/slamtec_c1_techdiffbot/bumperbot_ws
```

Launch our `AMCL` and `NAV2`
```
ros2 launch bumperbot_bringup autonomous_nav_sim.launch.py use_sim_time:=True world:=src/bumperbot_bringup/world/map_v1 
```

Launch `rviz`
```
rviz2 
```

and select the `rviz` configuration as follow
```
Fixed Frame - map
Map Topic - Global../Costmap
Color Scheme - Costmap
Durability Policy - Transient Local
Path - Topic - /plan
```

From `rviz`
```
select 2D Pose Estimate and select your initial position and please ensure it align with robot orientation.
This will generate a Global Costmap view
```

And from `rviz`
```
Select 2D Goal Pose and start navigating
```

And from `rviz` click `+` sign and add `GoalTool` from `nav2_rviz_plugins`
```
You can now use Nav2 Goal to move the robot
```


## Additional Notes

To display plot graph of `imu data`. You can run following command. Select topic and drag the parameter to graph view
```
ros2 run plotjuggler plotjuggler 
```

Moreover, you can use `rviz` to view the data of `imu`. Please ensure to install following packages first if you havent
```
sudo apt install ros-$ROS_DISTRO-plotjuggler-ros
sudo apt-get install ros-humble-rviz-imu-plugin
```

## Fusing IMU and Odometry using Kalman Filter and Create a new Filtered Odom as a new source of odometry

In this step, i'm going to show how to fuse `/imu/out` and `/diff_cont/odom` topic and create a new `/odometry/filtered` topic which is
coming from `/ekf_filter_node`.

We will use this `/odometry/filtered` in `SLAM` and `NAV` as new source of odometry.

Expected output will be as follow, and use this command to see all the listed topic
```
rqt 
```

and you will see result of rqt graph as follow. Here, you can see `1 - /diff_cont/odom`, `2 - /imut/out` are fused and generate an output `3 - /odometry/filtered` output coming
from `/ekf_filter_node`.

![image](https://github.com/user-attachments/assets/2ee5ab64-5daa-471f-a38f-041da23d98c6)

To enable the fuse, we have modify the following parameters in this file

* Edit `bumperbot_controllers.yaml` and make changes follow
```
enable_odom_tf: false
```

* Edit `gazebo.sim.launch.py` and enable this `launcher`
```
safety_stop
local_imu <----- This one
```

* Edit `bumperbot_gazebo.xacro` to point to our new controllers
```
<gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
        <!-- <parameters>$(find bumperbot_bringup)/config/bumperbot_bringup.yaml</parameters> -->
        <parameters>$(find bumperbot_bringup)/config/bumperbot_controllers.yaml</parameters> <----- This one
    </plugin>
</gazebo>
```
* Edit `ekf.yaml` and add/edit the following parameters. I highly suggest to refer to this `userguide` if you want
to know what does this file mean and how to create a new `filtered_odom`.

```
https://docs.nav2.org/setup_guides/odom/setup_odom.html#
```

Edit the following file and refer to above link to understand.

The order of the values of this parameter is `x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az`.

In our project, we set everything in odom0_config to false except the 1st, 2nd, 3rd, and 12th entries, which means
the filter will only use the x, y, z, and the vyaw values of odom0.

In the imu0_config matrix, you’ll notice that only roll, pitch, and yaw are used. 
```
two_d_mode: false
publish_tf: true
base_link_frame: base_footprint <-- Please ensure this match to our URDF and local_localization.launch.py

 # IMU Configuration
imu0: imu/out
imu0_config: [false, false, false,
            true,  true,  true,
            false, false, false,
            false, false, false,
            false, false, false]

# ODOM Configuration
odom0: diff_cont/odom
odom0_config: [true,  true,  true,
                false, false, false,
                false, false, false,
                false, false, true,
                false, false, false]
```

* Edit `nav2_params.yaml` and edit the following
```
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered # <----- Change this, default: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
```
