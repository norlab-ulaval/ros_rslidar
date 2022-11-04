/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"
#include <rslidar_msgs/msg/rslidar_scan.hpp>

namespace rslidar_driver
{
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 20000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;

rslidarDriver::rslidarDriver(std::shared_ptr<rclcpp::Node> node):
        node(node),
        diagnostics_(node)
{
  skip_num_ = 0;
  // get parameters
  node->declare_parameter<std::string>("frame_id", "rslidar");
  node->get_parameter("frame_id", config_.frame_id);

  // get model name, validate string, determine packet rate
  node->declare_parameter<std::string>("model", "RS16");
  node->get_parameter("model", config_.model);
  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16")
  {
    packet_rate = 840;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    packet_rate = 1690;
    model_full_name = "RS-LiDAR-32";
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "unknown LIDAR model: " << config_.model);
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  node->declare_parameter<double>("rpm", 600.0);
  node->get_parameter("rpm", config_.rpm);
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);
  node->declare_parameter<int>("npackets", npackets);
  node->get_parameter("npackets", config_.npackets);
  RCLCPP_INFO_STREAM(node->get_logger(), "publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  node->declare_parameter<std::string>("pcap", "");
  node->get_parameter("pcap", dump_file);

  int msop_udp_port;
  node->declare_parameter<int>("msop_port", (int)MSOP_DATA_PORT_NUMBER);
  node->get_parameter("msop_port", msop_udp_port);
  int difop_udp_port;
  node->declare_parameter<int>("difop_port", (int)DIFOP_DATA_PORT_NUMBER);
  node->get_parameter("difop_port", difop_udp_port);

  double cut_angle;
  node->declare_parameter<double>("cut_angle", -0.01);
  node->get_parameter("cut_angle", cut_angle);
  if (cut_angle < 0.0)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < 360)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Cut at specific angle feature activated. "
                    "Cutting rslidar points always at "
                    << cut_angle << " degree.");
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "cut_angle parameter is out of range. Allowed range is "
                     << "between 0.0 and 360 negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("rslidar_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                        TimeStampStatusParam()));

  // open rslidar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    msop_input_.reset(new rslidar_driver::InputPCAP(node, msop_udp_port, packet_rate, dump_file));
    difop_input_.reset(new rslidar_driver::InputPCAP(node, difop_udp_port, packet_rate, dump_file));
  }
  else
  {
    // read data from live socket
    msop_input_.reset(new rslidar_driver::InputSocket(node, msop_udp_port));
    difop_input_.reset(new rslidar_driver::InputSocket(node, difop_udp_port));
  }

  // raw packet output topic
  std::string output_packets_topic;
  node->declare_parameter<std::string>("output_packets_topic", "rslidar_packets");
  node->get_parameter("output_packets_topic", output_packets_topic);
  msop_output_ = node->create_publisher<rslidar_msgs::msg::RslidarScan>(output_packets_topic, 10);

  std::string output_difop_topic;
  node->declare_parameter<std::string>("output_difop_topic", "rslidar_packets_difop");
  node->get_parameter("output_difop_topic", output_difop_topic);

  difop_output_ = node->create_publisher<rslidar_msgs::msg::RslidarPacket>(output_difop_topic, 10);

  difop_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&rslidarDriver::difopPoll, this)));

  node->declare_parameter<bool>("time_synchronization", false);
  node->get_parameter("time_synchronization", time_synchronization_);

  if (time_synchronization_)
  {
    output_sync_ = node->create_publisher<sensor_msgs::msg::TimeReference>("sync_header", 1);
    skip_num_sub_ = node->create_subscription<std_msgs::msg::Int32>("skippackets_num", 1, std::bind(&rslidarDriver::skipNumCallback, this, std::placeholders::_1));
  }
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(void)
{  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  rslidar_msgs::msg::RslidarScan::SharedPtr scan(new rslidar_msgs::msg::RslidarScan);

  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);
    rslidar_msgs::msg::RslidarPacket tmp_packet;
    while (true)
    {
      while (true)
      {
        int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else  // standard behaviour
  {
    if (difop_input_->getUpdateFlag())
    {
      int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND/BLOCKS_ONE_CHANNEL_PER_PKT);
      int mode = difop_input_->getReturnMode();
      if (config_.model == "RS16" && (mode == 1))
      {
        packets_rate = ceil(packets_rate/2);
      }
      else if (config_.model == "RS32" && (mode == 0))
      {
        packets_rate = packets_rate*2;
      }
      config_.rpm = difop_input_->getRpm();
      config_.npackets = ceil(packets_rate*60/config_.rpm);

      difop_input_->clearUpdateFlag();
    }
    scan->packets.resize(config_.npackets);
    // use in standard behaviour only
    while (skip_num_)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[0], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      --skip_num_;
    }

    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
    }

    if (time_synchronization_)
    {
      sensor_msgs::msg::TimeReference sync_header;
      // it is already the msop msg
      // if (pkt->data[0] == 0x55 && pkt->data[1] == 0xaa && pkt->data[2] == 0x05 && pkt->data[3] == 0x0a)
      // use the first packets
      rslidar_msgs::msg::RslidarPacket pkt = scan->packets[0];
      struct tm stm;
      memset(&stm, 0, sizeof(stm));
      stm.tm_year = (int)pkt.data[20] + 100;
      stm.tm_mon  = (int)pkt.data[21] - 1;
      stm.tm_mday = (int)pkt.data[22];
      stm.tm_hour = (int)pkt.data[23];
      stm.tm_min  = (int)pkt.data[24];
      stm.tm_sec  = (int)pkt.data[25];
      double stamp_double = mktime(&stm) + 0.001 * (256 * pkt.data[26] + pkt.data[27]) +
                            0.000001 * (256 * pkt.data[28] + pkt.data[29]);
      sync_header.header.stamp = rclcpp::Time((int64_t)(stamp_double*1e9));

      output_sync_->publish(sync_header);
    }
  }

  // publish message using time of last packet read
  RCLCPP_DEBUG(node->get_logger(), "Publishing a full rslidar scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  msop_output_->publish(*scan);

  // notify diagnostics that a message has been published, updating its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.force_update();

  return true;
}

void rslidarDriver::difopPoll(void)
{
  // reading and publishing scans as fast as possible.
  rslidar_msgs::msg::RslidarPacket::SharedPtr difop_packet_ptr(new rslidar_msgs::msg::RslidarPacket);
  while (rclcpp::ok())
  {
    // keep reading
    rslidar_msgs::msg::RslidarPacket difop_packet_msg;
    int rc = difop_input_->getPacket(&difop_packet_msg, config_.time_offset);
    if (rc == 0)
    {
      // std::cout << "Publishing a difop data." << std::endl;
      RCLCPP_DEBUG(node->get_logger(), "Publishing a difop data.");
      *difop_packet_ptr = difop_packet_msg;
      difop_output_->publish(*difop_packet_ptr);
    }
    if (rc < 0)
      return;  // end of file reached?
    //rclcpp::spin_some(node); removed this line during ros2 migration, this made the node crash and it seems to be working without it
  }
}

// add for time synchronization
void rslidarDriver::skipNumCallback(const std_msgs::msg::Int32& skip_num)
{
  // std::cout << "Enter skipNumCallback: " << skip_num->data << std::endl;
  skip_num_ = skip_num.data;
}
}  // namespace rslidar_driver
