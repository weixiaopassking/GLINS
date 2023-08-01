/*
 * @Description: 
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-15
 */

#ifndef CLOUD_SUBSCRIBER_HPP_
#define CLOUD_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "../data/cloud_data.hpp"

namespace pub_ns
{
class CloudSub
{
  public:
    CloudSub() = delete;
    CloudSub(ros::NodeHandle &nh, std::string topic_name, const size_t buffer_size);
    void ParseData(std::deque<data_ns::CloudData> &cloud_data_queue);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;

    std::deque<data_ns::CloudData> _cloud_data_que;
};
} // namespace pub_ns

#endif