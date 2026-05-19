cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(octomap_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Library
add_library(octo_map SHARED
  src/octomap.cpp 
)

target_link_libraries(octo_map
  ${PCL_LIBRARIES}
  -loctomap
  -loctomath
  
)

ament_target_dependencies(octo_map
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL
  octomap_msgs
  octomap
  std_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Executable
add_executable(octomap_mapping_node src/main.cpp)
target_link_libraries(octomap_mapping_node octo_map -loctomath  )

ament_target_dependencies(octomap_mapping_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  PCL
  octomap_msgs
  octomap
  std_msgs
  Eigen3
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Install library
install(TARGETS octo_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS octomap_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export include directories
ament_export_include_directories(include)
ament_export_libraries(octo_map)
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions PCL geometry_msgs octomap_msgs octomap Eigen3 tf2 tf2_ros tf2_sensor_msgs)

ament_package()
