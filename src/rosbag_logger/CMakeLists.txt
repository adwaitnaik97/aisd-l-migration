cmake_minimum_required(VERSION 3.25.2)
project(rosbag_logger)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "-Wall -g ${CMAKE_CXX_FLAGS}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag_logger_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include dirs
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable AFTER interface generation
add_executable(rosbag_logger src/rosbag_logger.cpp)

# Link dependencies
ament_target_dependencies(rosbag_logger
  rclcpp
  std_msgs
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  yaml-cpp
  nlohmann_json
  rosbag_logger_interfaces
)

install(TARGETS rosbag_logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
