cmake_minimum_required(VERSION 3.8)
project(rbt_pd_cnt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  rbt_pd_cnt
  SHARED 
  src/robot_pd_cnt.cpp 
  src/real_pd_cnt.cpp
)
target_include_directories(
  rbt_pd_cnt PRIVATE include
)

ament_target_dependencies(rbt_pd_cnt
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  sensor_msgs
)

pluginlib_export_plugin_description_file(controller_interface rbt_pd_cnt.xml)


target_include_directories(${PROJECT_NAME} PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(
  TARGETS
  rbt_pd_cnt
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  sensor_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  rbt_pd_cnt
)

ament_package()
