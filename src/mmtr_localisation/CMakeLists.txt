cmake_minimum_required(VERSION 3.8)
project(mmtr_localisation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/zupt.py
  ${PROJECT_NAME}/zero_yaw.py
  ${PROJECT_NAME}/pose_buffer.py
  ${PROJECT_NAME}/imu_calibration.py
  ${PROJECT_NAME}/odom_covariance_override.py
  ${PROJECT_NAME}/magnetometer_frame_conversion.py
  ${PROJECT_NAME}/rewind_odom.py
  ${PROJECT_NAME}/yaw_correction.py


DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
