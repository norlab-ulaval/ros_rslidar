/* -*- mode: C++ -*- */
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
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread.hpp>
#include "input.h"

namespace rslidar_driver
{
class rslidarDriver
{
public:
  /**
 * @brief rslidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  rslidarDriver(std::shared_ptr<rclcpp::Node> node);

  ~rslidarDriver()
  {
  }

  bool poll(void);
  void difopPoll(void);

private:
  /// Callback for skip num for time synchronization
  void skipNumCallback(const std_msgs::msg::Int32& skip_num);

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  std::shared_ptr<rclcpp::Node> node;
  boost::shared_ptr<Input> msop_input_;
  boost::shared_ptr<Input> difop_input_;
  rclcpp::Publisher<rslidar_msgs::msg::RslidarScan>::SharedPtr msop_output_;
  rclcpp::Publisher<rslidar_msgs::msg::RslidarPacket>::SharedPtr difop_output_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr output_sync_;
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  boost::shared_ptr<boost::thread> difop_thread_;

  // add for time synchronization
  bool time_synchronization_;
  uint32_t skip_num_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr skip_num_sub_;
};

}  // namespace rslidar_driver

#endif
