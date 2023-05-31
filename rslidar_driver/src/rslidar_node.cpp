/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver node for the Robosense 3D LIDARs.
 */
#include <rclcpp/rclcpp.hpp>
#include "rsdriver.h"

class RSDriverNode : public rclcpp::Node
{
public:
  RSDriverNode():
    Node("rsdriver")
  {
  }
};

using namespace rslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, my_handler);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<RSDriverNode>();

  // start the driver
  rslidar_driver::rslidarDriver dvr(node);
  // loop until shut down or end of file
  while (rclcpp::ok() && dvr.poll())
  {
    rclcpp::spin_some(node);
  }

  return 0;
}
