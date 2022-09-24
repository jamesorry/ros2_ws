cmake_minimum_required(VERSION 3.5)
project(qt_publish)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(my_bringup REQUIRED)
# find_package(my_cpp_py_point_cloud2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)



set(DEPENDENCIES
  geometry_msgs
  rclcpp
  image_geometry
  map_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  my_robot_interfaces
  sensor_msgs
  my_bringup
)

add_executable(qt_publish
    src/main.cpp
    src/widget.cpp
    src/widget.h
    src/widget.ui
)

target_link_libraries(qt_publish Qt5::Widgets)

install(DIRECTORY 
  images
  doc_file
  UI
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  qt_publish
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/python_ui_test_01.py
  scripts/pyqt5_test_02.py
  scripts/SendMapImage.py
  scripts/test_point_cloud.py
  scripts/start_pyqt5_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
