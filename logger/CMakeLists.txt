cmake_minimum_required(VERSION 3.5)
project(logger)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)

#find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

#################
# Setup targets #
#################

add_library(logger SHARED
  src/logger.cpp)

ament_target_dependencies(logger
  "rclcpp"
  "rclcpp_components"
  "rosbag2_cpp"
  "geometry_msgs"
  "sensor_msgs"
  "image_transport")

rclcpp_components_register_nodes(logger "logger::Logger")

############
# Install ##
############

install(TARGETS
logger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()