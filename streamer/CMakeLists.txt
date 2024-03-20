cmake_minimum_required(VERSION 3.5)
project(streamer)

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
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

#################
# Setup targets #
#################

add_library(streamer SHARED
  src/streamer.cpp)

ament_target_dependencies(streamer
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "image_transport"
  "cv_bridge")

target_link_libraries(streamer 
	${OpenCV_LIBS})

rclcpp_components_register_nodes(streamer "streamer::Streamer")

############
# Install ##
############

install(TARGETS
streamer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()