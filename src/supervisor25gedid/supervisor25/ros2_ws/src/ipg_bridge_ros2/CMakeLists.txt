cmake_minimum_required(VERSION 3.5)
project(ipg_bridge_ros2)

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
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vehiclecontrol_msgs REQUIRED)



set(PID_INCLUDE_DIR
   ${CMAKE_CURRENT_SOURCE_DIR}/pid)

add_subdirectory(pid)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PID_INCLUDE_DIR}
)

add_executable(IPG_CONTROL_BRIDGE src/ros2package.cpp)
ament_target_dependencies(IPG_CONTROL_BRIDGE rclcpp nav_msgs ackermann_msgs vehiclecontrol_msgs)
target_link_libraries(IPG_CONTROL_BRIDGE pid ${catkin_LIBRARIES})
add_dependencies(IPG_CONTROL_BRIDGE pid)

install(TARGETS
  IPG_CONTROL_BRIDGE
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
