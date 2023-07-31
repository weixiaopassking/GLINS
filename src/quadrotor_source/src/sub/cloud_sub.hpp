/**
*****************************************************************************
*  Copyright (C), 2023-2026,wengang.niu
*  @file cloud_sub.hpp
*  @brief  点云信息接收
*  @author  wengang.niu
*  @date    2023/7/30
*  @version v0.2
*  @ref github.com/Little-Potato-1990/localization_in_auto_driving
*  @note
****************************************************************************
*/

#ifndef _CLOUD_SUB_HPP
#define _CLOUD_SUB_HPP

#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// common
#include "sensor_type/cloud_type.hpp"

namespace rossub_ns
{
class CloudSub
{
  public:
    CloudSub() = default;
    CloudSub(ros::NodeHandle &node_handle, const std::string topic_name, const size_t buffer_size = 10);
    void ParseData(std::deque<common_ns::CloudType> &cloud_msg_que);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::deque<common_ns::CloudType> _cloud_msg_que;
}; // class CloudSub
} // namespace rossub_ns

#endif //_CLOUD_SUB_HPP