cmake_minimum_required(VERSION 3.8)
project(zed_splitter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(zed_splitter_node src/zed_splitter_node.cpp)

# Link dependencies
ament_target_dependencies(zed_splitter_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV)

# Install the executable
install(TARGETS
  zed_splitter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()