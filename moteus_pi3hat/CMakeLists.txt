cmake_minimum_required(VERSION 3.8)
project(moteus_pi3hat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_library(
      moteus_pi3hat
      src/pi3hat.cc)

add_executable(
  moteus_pi3hat_tool
  src/pi3hat_tool.cc 
)

target_include_directories(moteus_pi3hat
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)



target_link_libraries(moteus_pi3hat
  bcm_host
)

target_link_libraries(moteus_pi3hat_tool
  moteus_pi3hat
)
      
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS moteus_pi3hat
  EXPORT export_moteus_pi3hat
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS
  moteus_pi3hat_tool 
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_moteus_pi3hat HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(moteus_pi3hat)
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
