cmake_minimum_required(VERSION 3.14)
project(carmaker_sensor_kit_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

