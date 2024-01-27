cmake_minimum_required(VERSION 3.5)
project(gimbal_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
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

#find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(gSDK REQUIRED)

include_directories(include)

#################
# Setup targets #
#################

add_library(gimbal_manager SHARED
  src/gimbal_manager.cpp)

ament_target_dependencies(gimbal_manager
  "rclcpp"
  "rclcpp_components"
  "gSDK")

rclcpp_components_register_nodes(gimbal_manager "gimbal_manager::GimbalManager")

############
# Install ##
############

install(TARGETS
gimbal_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()