cmake_minimum_required(VERSION 3.8)
project(detection_pipeline)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(camera_publisher_node
  src/camera_publisher_node.cpp
)

# Link libraries / dependencies
ament_target_dependencies(camera_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_publisher_node
    ${OpenCV_LIBS}
)

# Install executable
install(TARGETS
  camera_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()

