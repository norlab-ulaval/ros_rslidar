cmake_minimum_required(VERSION 2.8.3)
project(rslidar_pointcloud_stamped)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    rslidar_driver
    rslidar_msgs
    dynamic_reconfigure
    libpointmatcher_ros
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)
find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)
find_package(libpointmatcher REQUIRED)
include_directories(${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${pointmatcher_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/CloudNode.cfg)
add_subdirectory(src)
