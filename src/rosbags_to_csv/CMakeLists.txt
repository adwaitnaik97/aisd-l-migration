cmake_minimum_required(VERSION 3.25.2)
project(rosbags_to_csv)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall -g ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(SQLite3 REQUIRED)

# Enable Qt MOC and UI processing
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Include directories
include_directories(
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/MessagesHandler
)

# Add the executable
add_executable(rosbags_to_csv src/rosbags_to_csv.cpp)

# Link dependencies
ament_target_dependencies(rosbags_to_csv
  rclcpp
  std_msgs
  geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  yaml-cpp
)

target_link_libraries(rosbags_to_csv Qt5::Widgets SQLite::SQLite3)

# Install targets
install(TARGETS rosbags_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
