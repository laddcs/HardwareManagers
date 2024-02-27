cmake_minimum_required(VERSION 3.5)
project(ircamera_manager)

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
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include /usr/include/libirimager/)

#################
# Setup targets #
#################

add_library(ircamera_manager SHARED
  src/ircamera_manager.cpp)

ament_target_dependencies(ircamera_manager
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "image_transport")

target_link_libraries(ircamera_manager
  irimager
  irimageprocessing
  ircore
  udev
  usb-1.0)

rclcpp_components_register_nodes(ircamera_manager "ircamera_manager::IRCameraManager")

add_executable(manual_composition_test
  src/manual_composition.cpp)

target_link_libraries(manual_composition_test
ircamera_manager
irimager
irimageprocessing
ircore
udev
usb-1.0)

ament_target_dependencies(manual_composition_test
  "rclcpp"
  "rclcpp_components")

############
# Install ##
############

install(TARGETS
ircamera_manager
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