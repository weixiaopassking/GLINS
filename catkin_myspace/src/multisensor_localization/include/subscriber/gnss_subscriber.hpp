/*
 * @Description: 激光雷达数据订阅
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-15
 */

#ifndef   GNSS_SUBSCRIBER_HPP_
#define  GNSS_SUBSCRIBER_HPP_

//c++
#include <deque>
//ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
//gnss自定义数据类型
#include "../sensor_data/gnss_data.hpp"

namespace multisensor_localization
{
  class GnssSubscriber
  {
  public:
    GnssSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    GnssSubscriber() = default;
    void ParseData(std::deque<GnssData> &gnss_data_buff);

  private:
    void MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<GnssData> new_gnss_data_buff_;
  };
}
#endif