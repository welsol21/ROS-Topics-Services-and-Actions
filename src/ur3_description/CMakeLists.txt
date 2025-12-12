cmake_minimum_required(VERSION 3.8)
project(ur3_description)

# Ensure we use at least C99 / C++14 if needed
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

# Install all non-source resources (URDF, meshes, launch, config, models, rviz)
install(
  DIRECTORY config launch meshes models rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

