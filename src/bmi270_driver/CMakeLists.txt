cmake_minimum_required(VERSION 3.8)
project(bmi270_driver)

find_package(ament_cmake REQUIRED)

add_library(bmi270_api STATIC
  bmi2.c
  bmi270.c
)

target_include_directories(bmi270_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Install header files
install(FILES
  bmi2.h
  bmi2_defs.h
  bmi270.h
  DESTINATION include/bmi270_driver
)

# Install library
install(TARGETS bmi270_api
  EXPORT export_bmi270_api
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the export set so downstream packages can find it
install(EXPORT export_bmi270_api
  FILE bmi270_driverTargets.cmake
  NAMESPACE bmi270_driver::
  DESTINATION share/bmi270_driver/cmake
)

# Export for ament to generate bmi270_driverConfig.cmake
ament_export_targets(export_bmi270_api)
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
ament_export_libraries(bmi270_api)

ament_package()

