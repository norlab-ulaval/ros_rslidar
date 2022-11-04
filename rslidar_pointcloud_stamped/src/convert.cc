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

/** @brief Constructor. */
Convert::Convert(std::shared_ptr<rclcpp::Node> node) : data_(new rslidar_rawdata::RawData())
{
  data_->loadConfigFile(node);  // load lidar parameters

  // advertise output point cloud (before subscribing to input data)
  std::string output_points_topic;
  node->declare_parameter<std::string>("output_points_topic", "rslidar_points");
  node->get_parameter("output_points_topic", output_points_topic);
  output_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_points_topic, 10);

  // subscribe to rslidarScan packets
  std::string input_packets_topic;
  node->declare_parameter<std::string>("input_packets_topic", "rslidar_packets");
  node->get_parameter("input_packets_topic", input_packets_topic);
  rslidar_scan_ = node->create_subscription<rslidar_msgs::msg::RslidarScan>(input_packets_topic, 10, std::bind(&Convert::processScan, this, std::placeholders::_1));
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::msg::RslidarScan& scanMsg)
{
  //pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::msg::PointCloud2 outMsg;
  int height;
  int width;
  int num_of_points;
  bool is_dense;

  if (data_->model == "RS16")
  {
    height = 16;
    width = 24 * (int)scanMsg.packets.size();
    is_dense = false;
  }
  else if (data_->model == "RS32")
  {
    height = 32;
    width = 12 * (int)scanMsg.packets.size();
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

  outMsg.header.stamp = scanMsg.packets[0].stamp;   // The timestamp in the header of the scanMsg belongs to the last packet, not the first one...
  outMsg.header.frame_id = scanMsg.header.frame_id;
  outMsg.height = height;
  outMsg.width = width;
  outMsg.is_dense = is_dense;


  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg.packets.size(); ++i)
  {
    rclcpp::Time timeStamp(outMsg.header.stamp);
    data_->unpack_stamped(scanMsg.packets[i], x_vect, y_vect, z_vect, intensity_vect, time_offset_vect, timeStamp);
  }

  //TODO: Fill the message with data in the vectors:
  outMsg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(outMsg);
  // this call also resizes the data structure according to the given width, height and fields
  pcd_modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                                       "t", 1, sensor_msgs::msg::PointField::UINT32);

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

  output_->publish(outMsg);

}
}  // namespace rslidar_pointcloud
