# launch/bmi270_imu_calibrated.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your package's share directory
    # package_name must match the actual name of the ROS2 package here
    package_name = 'bmi270_imu_node'
    package_share_directory = get_package_share_directory(package_name)

    # Path to the calibration YAML file
    calibration_file_path = os.path.join(package_share_directory, 'config', 'calibration.yaml')

    return LaunchDescription([
        Node(
            package=package_name,
            executable='bmi270_imu_calibtd_node',
            name='bmi270_imu_calibtd_node',
            output='screen',
            # Pass the calibration parameters from the YAML file
            parameters=[calibration_file_path]
        )
    ])
