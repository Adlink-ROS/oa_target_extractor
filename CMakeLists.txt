cmake_minimum_required(VERSION 3.5)
project(oa_target_extractor)

# ROS2 Flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(object_analytics_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS
	common
)
#find_package(cv_bridge REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(image_transport REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/oa_target_extractor_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${object_msgs_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  #${cv_bridge_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rmw_implementation
  std_msgs
  sensor_msgs
  tf2_ros
  tf2
  object_analytics_msgs
  pcl_conversions
  visualization_msgs
  #cv_bridge
  #OpenCV
  #image_transport
  #realsense2
)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
