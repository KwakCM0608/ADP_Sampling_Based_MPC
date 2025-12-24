cmake_minimum_required(VERSION 3.8)
project(controller_cpp)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

# 메인 컨트롤러
add_executable(controller_manager_cpp
  src/controller_manager_cpp.cpp
  src/pp.cpp
  src/map.cpp
  src/lookup_steer_angle.cpp
  src/ftg.cpp
  src/hybrid.cpp
  src/mpc.cpp
)

target_include_directories(controller_manager_cpp PRIVATE include)

ament_target_dependencies(controller_manager_cpp
  rclcpp
  rcl_interfaces
  ament_index_cpp
  yaml-cpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  f110_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(controller_manager_cpp
  yaml-cpp
)

# SafetyMonitor 노드
add_executable(safety_monitor
  src/safety_monitor.cpp
)
ament_target_dependencies(safety_monitor
  rclcpp
  sensor_msgs
  std_msgs
)

# EscapePlanner 노드
add_executable(escape_planner
  src/escape_planner.cpp
)
ament_target_dependencies(escape_planner
  rclcpp
  std_msgs
  ackermann_msgs
  geometry_msgs
  f110_msgs
)

install(TARGETS
  controller_manager_cpp
  safety_monitor
  escape_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

