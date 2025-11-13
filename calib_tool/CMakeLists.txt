cmake_minimum_required(VERSION 3.5)
project(calib_tool)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcutils REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include 
  ${OpenCV_INCLUDE_DIRS})
  
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

#################
# Setup targets #
#################

add_library(calib_tool SHARED
  src/calib_tool.cpp)

ament_target_dependencies(calib_tool
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "px4_msgs"
  "image_transport"
  "cv_bridge")

target_link_libraries(calib_tool 
	${OpenCV_LIBS} )

rclcpp_components_register_nodes(calib_tool "calib_tool::calib_tool")

############
# Install ##
############

install(TARGETS
calib_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()