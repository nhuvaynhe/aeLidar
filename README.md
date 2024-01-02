# Overview
This repository contains code and resources for a LiDAR (Light Detection and Ranging) Simultaneous Localization and Mapping (SLAM) project. 
The goal of this project is to leverage LiDAR sensor data to create a map of an environment while simultaneously localizing the sensor within that map.

# Packages that we used
- sllidar_ros2: read the LiDAR data and display it on RVIZ2.
- XSENS Dot Library: function to work with Xsens Dot imu.
- robot_localization: fuse IMU data with encoder for better odometry.
- ros2_controllers: send the command to control the car and receive encoder data over serial.
- slam_toolbox: create a map of environments from LiDAR and Odometry data.

# Results
- Built environment in our lab: [youtube](https://youtu.be/gZVQmG-HcZ8?si=6335LGZMQ2-RkiJM)
- Single room: [youtube](https://youtu.be/0jHfGaMGygg?si=8-SV1q3gUJQbZOMz)
- Hallway: [youtube](https://youtu.be/eAF99G3UYoM?si=meU5C1zaLtjYV0xk)
