cmake_minimum_required(VERSION 3.8)
project(bmi270_imu_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O1")

# Set C++ version
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# This finds the bmi270_driver package, which should make its exported targets available
find_package(bmi270_driver REQUIRED)


# Add executable
add_executable(bmi270_imu_node src/bmi270_imu_node.cpp)

# Add executable for the FIFO node
add_executable(bmi270_fifo_node src/bmi270_fifo_node.cpp) 


# Link libraries
# When linking to a library exported by another ament package with a namespace,
# use the full namespaced target name.
# The bmi270_driver exports 'bmi270_api' with the namespace 'bmi270_driver::'.
target_link_libraries(bmi270_imu_node
  bmi270_driver::bmi270_api # Correctly link using the namespaced target
  # Add other system libraries if needed, e.g., for I2C functions
  # If 'i2c' refers to a system library like libi2c-dev, you might need:
  i2c  # Uncomment if you specifically need -li2c, often implicitly handled by ioctl/unistd headers
)
# Link libraries for the FIFO node
target_link_libraries(bmi270_fifo_node
  bmi270_driver::bmi270_api
)

# Link ROS2 dependencies using ament_target_dependencies
# This ensures that include paths and other necessary build flags are propagated
# for the basic node:
ament_target_dependencies(bmi270_imu_node
  rclcpp
  sensor_msgs
  bmi270_driver # This ensures the bmi270_driver package's exports are considered
)
# Link ROS2 dependencies for the FIFO node
ament_target_dependencies(bmi270_fifo_node
  rclcpp
  sensor_msgs
  bmi270_driver
)

# Install executables
install(TARGETS
  bmi270_imu_node
  DESTINATION lib/${PROJECT_NAME}
)
# FIFO node executable
install(TARGETS
  bmi270_fifo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (if your node also provides headers, typically not needed here)
# NOTE: The bmi270_driver package is already installing these headers.
# Duplicating here might cause issues or be redundant.
# Generally, a node package should not install headers from its dependencies.
# If these headers are ONLY used by your node's source file and are part of the
# Bosch API, then they are handled by bmi270_driver. If you modify/wrap them
# and want to expose them as part of *this* package, then install them.
# For now, I'm commenting this out as it's likely redundant.
# install(FILES
#   bmi2.h
#   bmi2_defs.h
#   bmi270.h
#   bmi270_config_file.h
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()

