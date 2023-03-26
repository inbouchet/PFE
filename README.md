# PFE

This README document introduces the end-of-study project by Laëtitia Cabrol and Bouchet Inès. The purpose of this project is to reproduce, in an accessible manner, communication between a drone and a robot.
We drew inspiration from what was accomplished with the Perseverance drone and the Ingenuity robot on Mars.

Detailed instructions for reproducing this project are available in the Robot and Drone branches.

## Equipment used

### Turtlebot Waffle PI

- LiDAR 360°
* Camera Raspberry Pi
+ Microcontroller Raspberry Pi
- Control card OpenCR
* Batteries LiPo 11.1 V
+ Wheels & engines

![ROBOTISTURTLEBOT3WAFFLEPIRPI42GB-1](https://user-images.githubusercontent.com/83234731/227781557-e0ab130a-abce-4f07-913c-98fe144f2c2d.jpg)


### Drone PX4 Vision Kit 1.5

- GPS M8N
* Batteries LiPo
+ Wifi antennas
- PixHawk 6C
* Basic companion computer
+ Depth camera
- Optical flux and infrared sensor
* Engines & propellers

![px4_vision_kit_hero](https://user-images.githubusercontent.com/83234731/227781569-8b670940-040a-4529-90f3-276be90a62c4.jpg)

### System 

Robot (Turtlebot 3 Waffle Pi)
```
Ubuntu 20.04
ROS Noetic
OpenCR
```

Drone (PX4 Vision Dev Kit 1.5)
```
Ubuntu 18.04
ROS Melodic
Occipital Structure Core ROS driver
```

### Process

The process of this project is broken down into three distinct stages. First, the drone detects a target, then transmits its position to the robot. Then, the robot bypasss the target according to the coordinates received.

<img width="294" alt="Capture d’écran 2023-03-26 à 16 13 47" src="https://user-images.githubusercontent.com/83234731/227781710-1eb139a3-bb7d-4661-9ca7-b802cdde5e45.png">
