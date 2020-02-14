/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
//#include <pcl_conversions/pcl_conversions.h>

namespace rslidar_pointcloud
{
std::string model;

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters
  private_nh.param("model", model, std::string("RS16"));

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  private_nh.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
  output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  private_nh.param("input_packets_topic", input_packets_topic, std::string("rslidar_packets"));
  rslidar_scan_ = node.subscribe(input_packets_topic, 10, &Convert::processScan, (Convert*)this,
                                 ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  // config_.time_offset = config.time_offset;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
{
  //pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 outMsg;
  int height;
  int width;
  int num_of_points;
  bool is_dense;

  if (model == "RS16")
  {
    height = 16;
    width = 24 * (int)scanMsg->packets.size();
    is_dense = false;
  }
  else if (model == "RS32")
  {
    height = 32;
    width = 12 * (int)scanMsg->packets.size();
    is_dense = false;
  }

  num_of_points = height*width;

  std::vector<float> x_vect(num_of_points);
  std::vector<float> y_vect(num_of_points);
  std::vector<float> z_vect(num_of_points);
  std::vector<float> intensity_vect(num_of_points);
  std::vector<uint32_t> time_offset_vect(num_of_points);


  //outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  //outPoints->header.frame_id = scanMsg->header.frame_id;
  //outPoints->clear();
  // process each packet provided by the driver

  outMsg.header.stamp = scanMsg->packets[0].stamp;   // The timestamp in the header of the scanMsg belongs to the last packet, not the first one...
  outMsg.header.frame_id = scanMsg->header.frame_id;
  outMsg.height = height;
  outMsg.width = width;
  outMsg.is_dense = is_dense;


  std::cout << "-------------------------------------" << std::endl;

  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack_stamped(scanMsg->packets[i], x_vect, y_vect, z_vect, intensity_vect, time_offset_vect, outMsg.header.stamp);
  }

  //TODO: Fill the message with data in the vectors:
  outMsg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(outMsg);
  // this call also resizes the data structure according to the given width, height and fields
  pcd_modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::PointField::FLOAT32,
                                       "intensity", 1, sensor_msgs::PointField::FLOAT32,
                                       "t", 1, sensor_msgs::PointField::UINT32);

  sensor_msgs::PointCloud2Iterator<float> iter_x(outMsg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(outMsg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(outMsg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(outMsg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(outMsg, "t");


  int index_in_vectors;
  for (index_in_vectors = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_t, ++index_in_vectors)
  {
      // copy the data
      *iter_x = x_vect[index_in_vectors];
      *iter_y = y_vect[index_in_vectors];
      *iter_z = z_vect[index_in_vectors];
      *iter_i = intensity_vect[index_in_vectors];
      *iter_t = time_offset_vect[index_in_vectors];

  }

  output_.publish(outMsg);

}
}  // namespace rslidar_pointcloud
