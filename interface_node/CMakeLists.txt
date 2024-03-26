cmake_minimum_required(VERSION 3.8)
project(interface_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moteus_pi3hat REQUIRED)

add_executable(interface_node_v1 
                src/int_node_v1.cpp)
add_executable(raw_data 
                src/imu_raw_data.cpp)

target_link_libraries(raw_data
                      bcm_host
                      )
target_link_libraries(interface_node_v1
                  bcm_host
                      )
ament_target_dependencies(interface_node_v1 rclcpp std_msgs sensor_msgs moteus_pi3hat)
ament_target_dependencies(raw_data rclcpp std_msgs sensor_msgs moteus_pi3hat)
install(TARGETS
  interface_node_v1 
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  raw_data 
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
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
