cmake_minimum_required(VERSION 3.8)
project(pi3hat_omni_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pi3hat_hw_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/pi3hat_vel_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)


ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  pi3hat_hw_interface
  pi3hat_moteus_int_msgs
  std_srvs
  Eigen3
  sensor_msgs
  
)


install(
  TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)



pluginlib_export_plugin_description_file(controller_interface pi3hat_omni_controller.xml)

install(
  DIRECTORY launch urdf config
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

ament_export_dependencies(
  builtin_interfaces
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  pi3hat_hw_interface
  pi3hat_moteus_int_msgs
  std_srvs
  Eigen3
  sensor_msgs
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()
