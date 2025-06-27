# imu_bmi270_ros2

ROS2 package to use with the BMI270 

A WIP.

This makes use of Bosch's API for their BMI270, see:

https://github.com/boschsensortec/BMI270_SensorAPI/tree/master

Some code from this repository have been included here. Lots of example are also available on bosch's repo.


## install

Depends on ros2 (tested on jazzy and a raspberrypi 5), 

mkdir -p ~/ros2_ws/imu_bmi270_ros2

git clone the repo src into dir above

so that you get the following tree:

└── src
    ├── bmi270_driver
    └── bmi270_imu_node



build:

    $pwd
    ~/ros2_ws/imu_bmi270_ros2

    colcon build

    source install/setup.bash

## use

    ros2 run bmi270_imu_node bmi270_imu_node


You should then see the topic being publish:

    ros2 topic list

    ros2 topic echo /imu/data_raw

## python

Not fully working but it's there.

