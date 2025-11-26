cmake_minimum_required(VERSION 3.5)
project(tf_nova)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tf_nova_node src/tf_nova_node.cpp)
ament_target_dependencies(tf_nova_node rclcpp sensor_msgs)

install(TARGETS tf_nova_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
