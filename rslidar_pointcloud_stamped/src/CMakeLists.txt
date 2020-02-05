add_library(rslidar_data_stamped rawdata.cc)
target_link_libraries(rslidar_data_stamped
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    ${POINTMATCHER_LIBRARIES}
    )

add_library(rslidar_point_stamped convert.cc)
target_link_libraries(rslidar_point_stamped
    rslidar_data_stamped
    ${catkin_LIBRARIES}
    ${POINTMATCHER_LIBRARIES})


#add_library(cloud_nodelet cloud_nodelet.cc)
#target_link_libraries(cloud_nodelet rslidar_point
#                      ${catkin_LIBRARIES})

add_executable(cloud_node_stamped cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_data_stamped ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(cloud_node_stamped
    rslidar_point_stamped
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    ${POINTMATCHER_LIBRARIES})
