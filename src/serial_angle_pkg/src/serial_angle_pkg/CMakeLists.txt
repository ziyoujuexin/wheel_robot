cmake_minimum_required(VERSION 3.8)
project(serial_angle_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)  # 添加 std_msgs 依赖

# 查找 serial 库
find_package(serial REQUIRED)  # 或者使用 pkg-config 方式，具体取决于您的安装方式

find_package(rclcpp REQUIRED)
add_executable(serial_angle_node src/serial_angle_node.cpp)
ament_target_dependencies(serial_angle_node rclcpp std_msgs serial)
install(TARGETS
serial_angle_node
DESTINATION lib/${PROJECT_NAME}
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
