/*
 * @Description: cloud subscriber
 * @Function: 
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _CLOUD_SUB_HPP
#define _CLOUD_SUB_HPP

//system
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//data
#include "../data/cloud_data.hpp"

namespace sub_ns
{
class CloudSub
{
  public:
    CloudSub() = delete;//must  set node handle
    CloudSub(ros::NodeHandle &nh, const std::string  &topic_name, const size_t buffer_size);
    void ParseData(std::deque<data_ns::CloudData> &cloud_data_queue);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    std::deque<data_ns::CloudData> _cloud_data_que;
}; // class CloudSub
} // namespace pub_ns

#endif //_CLOUD_SUB_HPP