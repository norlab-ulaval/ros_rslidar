cmake_minimum_required(VERSION 3.8)
project(rslidar_pointcloud_stamped)

set(CMAKE_BUILD_TYPE Release)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rslidar_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(rslidar_data_stamped src/rawdata.cc)
ament_target_dependencies(rslidar_data_stamped
  rclcpp
  rslidar_msgs
  pcl_ros
  pcl_conversions
)

add_library(rslidar_point_stamped src/convert.cc)
ament_target_dependencies(rslidar_point_stamped
  rclcpp
  sensor_msgs
)
target_link_libraries(rslidar_point_stamped
  rslidar_data_stamped
)

#add_library(cloud_nodelet cloud_nodelet.cc)
#target_link_libraries(cloud_nodelet rslidar_point
#  ${catkin_LIBRARIES}
#)

add_executable(cloud_node_stamped src/cloud_node.cc)
target_link_libraries(cloud_node_stamped
  rslidar_point_stamped
)

install(TARGETS cloud_node_stamped
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
