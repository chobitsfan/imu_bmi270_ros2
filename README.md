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

Connect the IMU following the dtoverlay setting in /boot/firmware/config.txt.

Tested with the IMU connected to i2c bus 1 (i2c1) and i2c bus 3.

Add the following to  config.txt if connecting the BMI270 to i2c bus 1, with SDA and SCL on GPIO pins 2 and 3 (physical pins 3 and 5), respectively.

    dtparam=i2c_arm=on

Or enable i2c in raspi-config

Tip: enable a given i2c bus at run time without changing config.txt. There's no specific additional setup for the ic2 bus 1 besides enabling i2c. 
The following enables i2c bus 3 on GPIO pins 22 and 23.

    $ sudo dtoverlay i2c3-pi5 pins_22_23

For now if the bus and addresses of the IMU are hardcoded in the node source code, and need to be adjusted accordingly.

See `i2c_fd_ = open("/dev/i2c-1", O_RDWR); # change e.g. i2c-3` 

    ros2 run bmi270_imu_node bmi270_imu_node


You should then see the topic being publish:

    ros2 topic list

    ros2 topic echo /imu/data_raw

## python

Not fully working but it's there.

