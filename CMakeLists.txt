cmake_minimum_required(VERSION 3.8)
project(opus_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add include directories
include_directories(include)

# Source files for the project
set(SRC_FILES
  src/utils/utils.cpp
  src/motor_protocols/rs03.cpp
)

# Create the library
add_library(${PROJECT_NAME}_lib ${SRC_FILES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Bring in ROS 2 dependencies for the library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  can_msgs
  tf2_ros
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Optionally, add the test node as an executable
add_executable(test_rs03_node src/test_rs03_node.cpp)
ament_target_dependencies(test_rs03_node
  rclcpp
  can_msgs
  tf2_ros
)
target_link_libraries(test_rs03_node
  ${PROJECT_NAME}_lib
)

install(TARGETS test_rs03_node
  DESTINATION lib/${PROJECT_NAME}
)

# Testing (if needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
