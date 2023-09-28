/*
 * @Description: cloud subscriber
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-09-28
 */

#ifndef _CLOUD_SUB_HPP
#define _CLOUD_SUB_HPP

#include "../msg/cloud_data.hpp"
#include "sub_base.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace sub
{
class CloudSub : public SubBase
{
  public:
    CloudSub();
    CloudSub(const ros::NodeHandle &nh, const std::string &topic_name, const size_t buffer_size);
    ~CloudSub();
    void ParseData(std::deque<msg::CloudData> &cloud_data_queue);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    std::deque<msg::CloudData> _cloud_data_que;
}; // class CloudSub
} // namespace sub

#endif //_CLOUD_SUB_HPP