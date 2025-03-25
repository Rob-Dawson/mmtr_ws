cmake_minimum_required(VERSION 3.8)
project(mmtr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(urdf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
${PROJECT_NAME}/gz_tf_bridge.py
DESTINATION lib/${PROJECT_NAME})

# add_executable(flipper_controller src/flipper_controller.cpp)
# ament_target_dependencies(flipper_controller rclcpp std_msgs sensor_msgs trajectory_msgs)
include_directories(include)

add_executable(mapping src/mapping.cpp)


# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})
ament_target_dependencies(mapping rclcpp sensor_msgs tf2_ros tf2 nav_msgs geometry_msgs)


add_executable(imu_republisher src/imu_repub.cpp)
ament_target_dependencies(imu_republisher rclcpp sensor_msgs)

install(TARGETS
  imu_republisher
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch meshes worlds rviz urdf config
  DESTINATION share/${PROJECT_NAME}
)

# install(TARGETS
#   flipper_controller
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
