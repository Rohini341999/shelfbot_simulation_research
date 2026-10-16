cmake_minimum_required(VERSION 3.8)
project(shelfbot_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executables
install(PROGRAMS
  shelfbot_sim/shelf_detector.py
  shelfbot_sim/climb_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
