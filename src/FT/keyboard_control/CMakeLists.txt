cmake_minimum_required(VERSION 3.8)
project(keyboard_control)

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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(scservo REQUIRED)
find_package(ftmsg REQUIRED)
# if(scservo_FOUND)
#     message ("boost found")
#     include_directories(${Boost_INCLUDE_DIRS})
# else()
#     message (FATAL_ERROR "Cannot find Boost")
# endif()

# add_library(keyboard_control src/keyboard_control.cpp)
# message(STATUS "scservo_FOUND = ${scservo_FOUND}")
# message(STATUS "scservo_DIR = ${scservo_DIR}")
add_executable(${PROJECT_NAME} src/keyboard_control.cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${PROJECT_NAME}
  scservo
  rclcpp
  std_msgs
  ftmsg
)

# target_link_libraries(${PROJECT_NAME} scservo)

# target_link_libraries(${PROJECT_NAME} "~/wheeltec_ros2/src/FT/FTmotor/install/scservo/lib/libscservo.so")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(keyboard_control PRIVATE "KEYBOARD_CONTROL_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS keyboard_control
  EXPORT export_${PROJECT_NAME}

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

ament_export_include_directories(
  include
)
# ament_export_libraries(
#   keyboard_control
# )
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
