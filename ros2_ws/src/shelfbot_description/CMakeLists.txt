cmake_minimum_required(VERSION 3.8)
project(shelfbot_description)

find_package(ament_cmake REQUIRED)

# Install all package directories so they are reachable via package:// URIs
install(
  DIRECTORY meshes urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
