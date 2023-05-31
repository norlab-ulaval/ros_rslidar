/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */
#include "input.h"

extern volatile sig_atomic_t flag;
namespace rslidar_driver
{
static const size_t packet_size = sizeof(rslidar_msgs::msg::RslidarPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(std::shared_ptr<rclcpp::Node> node, uint16_t port) : node(node), port_(port)
{
  npkt_update_flag_ = false;
  cur_rpm_ = 600;
  return_mode_ = 1;

  if(!node->has_parameter("device_ip"))
    node->declare_parameter<std::string>("device_ip", "");
  node->get_parameter("device_ip", devip_str_);
  if (!devip_str_.empty())
    RCLCPP_INFO_STREAM(node->get_logger(), "Only accepting packets from IP address: " << devip_str_);
}

int Input::getRpm(void)
{
  return cur_rpm_;
}

int Input::getReturnMode(void)
{
  return return_mode_;
}

bool Input::getUpdateFlag(void)
{
  return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
  npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(std::shared_ptr<rclcpp::Node> node, uint16_t port) : Input(node, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one rslidar packet. */
int InputSocket::getPacket(rslidar_msgs::msg::RslidarPacket* pkt, const double time_offset)
{
  double time1 = node->get_clock()->now().seconds();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
          RCLCPP_ERROR(node->get_logger(), "poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
        RCLCPP_WARN(node->get_logger(), "Rslidar poll() timeout");

        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);          // initialize to zeros
        sender_address.sin_family = AF_INET;                     // host byte order
        sender_address.sin_port = htons(MSOP_DATA_PORT_NUMBER);  // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        RCLCPP_ERROR(node->get_logger(), "poll() reports Rslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        RCLCPP_INFO(node->get_logger(), "recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
        continue;
      else
        break;  // done
    }

    RCLCPP_DEBUG_STREAM(node->get_logger(), "incomplete rslidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }

  if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
  {//difop
    int rpm = (pkt->data[8]<<8)|pkt->data[9];
    int mode = 1;

    if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08)
        || (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
    {
      if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
      {
        mode = 0;
      }
    }

    if (cur_rpm_ != rpm || return_mode_ != mode)
    {
      cur_rpm_ = rpm;
      return_mode_ = mode;

      npkt_update_flag_ = true;
    }
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = node->get_clock()->now().seconds();
  pkt->stamp = rclcpp::Time((int64_t)(((time2 + time1) / 2.0 + time_offset)*1e9));

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
InputPCAP::InputPCAP(std::shared_ptr<rclcpp::Node> node, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(node, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;

  // get parameters using private node handle
  if(!node->has_parameter("read_once"))
    node->declare_parameter<bool>("read_once", false);
  node->get_parameter("read_once", read_once_);
  if(!node->has_parameter("read_fast"))
    node->declare_parameter<bool>("read_fast", false);
  node->get_parameter("read_fast", read_fast_);
  if(!node->has_parameter("repeat_delay"))
    node->declare_parameter<double>("repeat_delay", 0.0);
  node->get_parameter("repeat_delay", repeat_delay_);

  if (read_once_)
    RCLCPP_INFO(node->get_logger(), "Read input file only once.");
  if (read_fast_)
    RCLCPP_INFO(node->get_logger(), "Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    RCLCPP_INFO(node->get_logger(), "Delay %.3f seconds before repeating input file.", repeat_delay_);

  // Open the PCAP dump file
  // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening PCAP file " << filename_);
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    RCLCPP_FATAL(node->get_logger(), "Error opening rslidar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one rslidar packet. */
int InputPCAP::getPacket(rslidar_msgs::msg::RslidarPacket* pkt, const double time_offset)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

  while (flag == 1)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();


      memcpy(&pkt->data[0], pkt_data + 42, packet_size);

      if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
      {//difop
        int rpm = (pkt->data[8]<<8)|pkt->data[9];
        int mode = 1;

        if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08)
            || (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
        {
          if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
          {
            mode = 0;
          }
        }

        if (cur_rpm_ != rpm || return_mode_ != mode)
        {
          cur_rpm_ = rpm;
          return_mode_ = mode;

          npkt_update_flag_ = true;
        }
      }

      pkt->stamp = node->get_clock()->now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      RCLCPP_WARN(node->get_logger(), "Error %d reading rslidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      RCLCPP_INFO(node->get_logger(), "end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      RCLCPP_INFO(node->get_logger(), "end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    RCLCPP_DEBUG(node->get_logger(), "replaying rslidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again

  if (flag == 0)
  {
    abort();
  }
}
}
