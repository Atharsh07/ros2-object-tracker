cmake_minimum_required(VERSION 3.5)
project(ros2_object_tracker)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ros2_object_tracker/object_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
