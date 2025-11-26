cmake_minimum_required(VERSION 3.8)
project(ft_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scservo REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ftmsg REQUIRED)

add_library(ft_node SHARED src/ftmotor_base_node.cpp)
# add_library(ft_base STATIC src/subnode_ft.cpp)
add_executable(subnode_ft src/subnode_ft.cpp)
add_executable(ft_publisher src/ftpublisher.cpp)

add_library(ft_driver::ft_node ALIAS ft_node)

target_link_libraries(ft_publisher ft_node)
target_link_libraries(subnode_ft ft_node)


ament_target_dependencies(
  subnode_ft
  rclcpp
  scservo
  std_msgs
  ftmsg
)

ament_target_dependencies(
  ft_publisher
  rclcpp
  std_msgs
  scservo
  ftmsg
)

ament_target_dependencies(
  ft_node
  rclcpp
  scservo
  std_msgs
  ftmsg
)
# target_include_directories(
#   ft_base PUBLIC 
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
#   $<INSTALL_INTERFACE:include>
# )


target_include_directories(
  subnode_ft PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_include_directories(
  ft_publisher PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_include_directories(
  ft_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

#install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  ft_node
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
  subnode_ft
  ft_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME}
)


ament_export_dependencies(
  rclcpp
  scservo
  std_msgs
  ftmsg
)

ament_export_libraries(ft_node)

ament_package()

