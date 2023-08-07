/*
 * @Description: gnss subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#ifndef _GNSS_SUB_HPP
#define _GNSS_SUB_HPP

//system
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
//data
#include "../data/gnss_data.hpp"

namespace sub_ns
{
class GNSSSub
{
  public:
    GNSSSub() = delete;//must set node handle
    GNSSSub(ros::NodeHandle &nh, const std::string &topic_name, const size_t buffer_size);
    void ParseData(std::deque<data_ns::GNSSData> &gnss_data_deq);

  private:
    void MsgCallback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    std::deque<data_ns::GNSSData> _gnss_data_deq;

};//class GNSSSub 
} // namespace multisensor_localization
#endif //_GNSS_SUB_HPP