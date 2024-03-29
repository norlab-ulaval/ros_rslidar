/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Robosense 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *  @author Tony Zhang
 */

#ifndef _RAWDATA_H
#define _RAWDATA_H

#include <rclcpp/rclcpp.hpp>
#include <rslidar_msgs/msg/rslidar_packet.hpp>
#include <rslidar_msgs/msg/rslidar_scan.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>



namespace rslidar_rawdata
{
// static const float  ROTATION_SOLUTION_ = 0.18f;  //水平角分辨率 10hz
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);  // 96

static const float ROTATION_RESOLUTION = 0.01f;   /**< degrees 旋转角分辨率*/
static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

static const float DISTANCE_MAX = 200.0f;            /**< meters */
static const float DISTANCE_MIN = 0.2f;              /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);
/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;  //
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for RS16 support **/
static const int RS16_FIRINGS_PER_BLOCK = 2;
static const int RS16_SCANS_PER_FIRING = 16;
static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]

/** Special Defines for RS32 support **/
static const int RS32_FIRINGS_PER_BLOCK = 1;
static const int RS32_SCANS_PER_FIRING = 32;
static const float RS32_BLOCK_TDURATION = 50.0f;  // [µs]
static const float RS32_DSR_TOFFSET = 3.0f;       // [µs]
static const float RL32_FIRING_TOFFSET = 50.0f;   // [µs]

static const int TEMPERATURE_MIN = 31;

/** \brief Raw rslidar data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
// block
typedef struct raw_block
{
  uint16_t header;  ///< UPPER_BANK or LOWER_BANK
  uint8_t rotation_1;
  uint8_t rotation_2;  /// combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];  // 96
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes
{
  uint16_t uint;
  uint8_t bytes[2];
};

static const int PACKET_SIZE = 1248;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw Rsldar packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet
{
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint16_t revolution;
  uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

/** \brief RSLIDAR data conversion class */
class RawData
{
public:
  RawData();

  ~RawData()
  {
  }

  /*load the cablibrated files: angle, distance, intensity*/
  void loadConfigFile(std::shared_ptr<rclcpp::Node> node);

  /*unpack the RS16 UDP packet and opuput PCL PointXYZI type*/
  void unpack(const rslidar_msgs::msg::RslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*unpack the RS16 UDP packet and output several std::vectors with the values*/
  void unpack_stamped(const rslidar_msgs::msg::RslidarPacket& pkt,
                      std::vector<float>&  x_vect,
                      std::vector<float>&  y_vect,
                      std::vector<float>&  z_vect,
                      std::vector<float>&  intensity_vect,
                      std::vector<uint32_t>&  time_offset_vect,
                      rclcpp::Time& first_stamp
                      );

  /*unpack the RS32 UDP packet and output PCL PointXYZI type*/
  void unpack_RS32(const rslidar_msgs::msg::RslidarPacket& pkt, pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);

  /*unpack the RS32 UDP packet and output several std::vectors with the values*/
  void unpack_RS32_stamped(const rslidar_msgs::msg::RslidarPacket& pkt,
                           std::vector<float>&  x_vect,
                           std::vector<float>&  y_vect,
                           std::vector<float>&  z_vect,
                           std::vector<float>&  intensity_vect,
                           std::vector<uint32_t>&  time_offset_vect,
                           rclcpp::Time& first_stamp
                           );


  /*compute temperature*/
  float computeTemperature(unsigned char bit1, unsigned char bit2);

  /*estimate temperature*/
  int estimateTemperature(float Temper);

  /*calibrated the disctance*/
  float pixelToDistance(int pixelValue, int passageway);

  /*calibrated the azimuth*/
  int correctAzimuth(float azimuth_f, int passageway);

  /*calibrated the intensity*/
  float calibrateIntensity(float inten, int calIdx, int distance);
  float calibrateIntensity_old(float inten, int calIdx, int distance);

  /*estimate the packet type*/
  int isABPacket(int distance);

  void processDifop(const rslidar_msgs::msg::RslidarPacket& difop_msg);
  rclcpp::Subscription<rslidar_msgs::msg::RslidarPacket>::SharedPtr difop_sub_;
  bool is_init_curve_;
  bool is_init_angle_;
  bool is_init_top_fw_;
  int block_num = 0;
  int intensity_mode_;
  int intensityFactor;
  std::string model;

private:
  float R1_;
  float R2_;
  bool angle_flag_;
  float start_angle_;
  float end_angle_;
  float max_distance_;
  float min_distance_;
  int dis_resolution_mode_;
  int return_mode_;
  bool info_print_flag_;
  std::shared_ptr<rclcpp::Node> node;
};

}  // namespace rslidar_rawdata

#endif  // __RAWDATA_H
