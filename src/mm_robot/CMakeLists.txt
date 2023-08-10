cmake_minimum_required(VERSION 3.8)
project(mm_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED)
find_package(boost_thread REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

add_executable(tarkbot src/tarkbot.cpp)
target_link_libraries(tarkbot ${Boost_LIBRARIES} boost_thread)
ament_target_dependencies(tarkbot rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros)

install(TARGETS
        tarkbot
        DESTINATION lib/${PROJECT_NAME})

add_library(
  diffdrive
  SHARED
  src/diffdrive.cpp
)
target_compile_features(diffdrive PUBLIC cxx_std_17)
target_include_directories(diffdrive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/xtarkr20>
)
ament_target_dependencies(
  diffdrive PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS diffdrive
  EXPORT export_diffdrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
