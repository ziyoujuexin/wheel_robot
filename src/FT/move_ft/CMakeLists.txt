cmake_minimum_required(VERSION 3.8)
project(move_ft)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scservo REQUIRED)
find_package(ft_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ftmsg REQUIRED)
find_package(wheeltec_mic_msg REQUIRED)

add_executable(motorModel src/motorModel.cpp)

target_link_libraries(motorModel ft_driver::ft_node)

ament_target_dependencies(
  motorModel
  rclcpp
  std_msgs
  ft_driver
  scservo
  ftmsg
  wheeltec_mic_msg
  geometry_msgs
)

target_include_directories(
  motorModel PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  motorModel
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_package()
