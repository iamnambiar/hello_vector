cmake_minimum_required(VERSION 3.8)
project(vector_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Collect SDK source files
file(GLOB SCSERVO_SRC FTServo_Linux/src/*.cpp)

# Build shared library: our code + SDK code
add_library(vector_hardware SHARED
  src/vector_hardware_interface.cpp
  ${SCSERVO_SRC}
)

# Where to find header files
target_include_directories(vector_hardware PRIVATE
  include
  FTServo_Linux/src
)

# Link against ROS2 libraries
ament_target_dependencies(vector_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Register plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface vector_hardware_plugin.xml)

# Install library and launch files
install(TARGETS vector_hardware DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
