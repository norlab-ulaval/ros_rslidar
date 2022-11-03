cmake_minimum_required(VERSION 3.8)
project(rslidar_driver)

set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rslidar_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(libpcap_LIBRARIES -lpcap)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${Boost_INCLUDE_DIRS})

add_library(rslidar_input src/input.cc)
ament_target_dependencies(rslidar_input
  rclcpp
  rslidar_msgs
  sensor_msgs
)
target_link_libraries(rslidar_input
  ${libpcap_LIBRARIES}
)

add_library(rslidar_driver src/rsdriver.cpp)
ament_target_dependencies(rslidar_driver
  rclcpp
  std_msgs
  diagnostic_updater
  pcl_ros
  pcl_conversions
  rslidar_msgs
)
target_link_libraries(rslidar_driver
  rslidar_input
  ${Boost_LIBRARIES}
)

# build the nodelet version
#add_library(driver_nodelet src/nodelet.cc src/rsdriver.cpp)
#target_link_libraries(driver_nodelet
#  rslidar_input
#  ${catkin_LIBRARIES}
#)

add_executable(rslidar_node src/rslidar_node.cpp)
ament_target_dependencies(rslidar_node
  rclcpp
)
target_link_libraries(rslidar_node
  rslidar_driver
)

install(TARGETS rslidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
