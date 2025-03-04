cmake_minimum_required(VERSION 3.8)
project(mmtr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(flipper_controller src/flipper_controller.cpp)
ament_target_dependencies(flipper_controller rclcpp std_msgs sensor_msgs trajectory_msgs)


install(
  DIRECTORY launch meshes worlds rviz urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  flipper_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
