cmake_minimum_required(VERSION 3.8)
project(mmtr_trajectories)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/square_teleop.py
  ${PROJECT_NAME}/circle_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
