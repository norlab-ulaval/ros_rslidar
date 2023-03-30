cmake_minimum_required(VERSION 3.8)
project(rslidar_msgs)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RslidarPacket.msg"
    "msg/RslidarScan.msg"
    DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()

