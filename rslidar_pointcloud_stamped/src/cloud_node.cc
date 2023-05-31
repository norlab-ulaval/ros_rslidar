/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include "convert.h"

class CloudNode : public rclcpp::Node
{
public:
  CloudNode():
    Node("cloud_node")
  {
  }
};

/** Main node entry point. */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<CloudNode>();

  // create conversion class, which subscribes to raw data
  rslidar_pointcloud::Convert conv(node);

  // handle callbacks until shut down
  rclcpp::spin(node);

  return 0;
}
