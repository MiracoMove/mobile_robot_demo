cmake_minimum_required(VERSION 3.8)
project(mm_gps_nav_gps_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(NemaTode_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(aros2_lifecycle REQUIRED)
find_package(aros2_core REQUIRED)
find_package(aros2_hardware REQUIRED)
find_package(Boost REQUIRED)
find_package(boost_thread REQUIRED)
find_package(NemaTode REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  lifecycle_msgs
  aros2_lifecycle
  aros2_core
  aros2_hardware
)

add_executable(sim_gps_controller src/sim_gps_controller.cpp)
ament_target_dependencies(sim_gps_controller 
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  lifecycle_msgs
  aros2_lifecycle
  aros2_core
  aros2_hardware
  tf2
)

add_executable(wit_gps_controller src/wit_gps_controller.cpp)
target_link_libraries(wit_gps_controller ${Boost_LIBRARIES} boost_thread NemaTode::NemaTode)
ament_target_dependencies(wit_gps_controller 
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  lifecycle_msgs
  tf2
  aros2_lifecycle
  aros2_core
  aros2_hardware
)

install(TARGETS
        sim_gps_controller
        wit_gps_controller
        DESTINATION lib/${PROJECT_NAME})


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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
