cmake_minimum_required(VERSION 3.5)
project(my_robot_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#if you add your own msg or srv file, make sure you add this line
find_package(rosidl_default_generators REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "msg/LedStates.msg"
  "msg/Turtle.msg"
  "msg/TurtleArray.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  # "msg/ObjectCount.msg"
  "srv/ComputeRectangleArea.srv"
  "srv/SetLed.srv"
  "srv/CatchTurtle.srv"
  "srv/TurnCamera.srv"
  "action/Navigate2D.action"
  DEPENDENCIES sensor_msgs geometry_msgs
)


ament_package()
