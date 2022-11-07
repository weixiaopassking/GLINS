/*
 * @Description: 激光雷达数据订阅
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-15
 */

#ifndef CLOUD_SUBSCRIBER_HPP_
#define CLOUD_SUBSCRIBER_HPP_

//c++
#include <deque>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//自定义点云数据类型
#include "../sensor_data/cloud_data.hpp"

namespace multisensor_localization
{
  class CloudSubscriber
  {
  public:
    CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    CloudSubscriber() = default;
    void ParseData(std::deque<CloudData> &cloud_data_buff);

  private:
    void MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<CloudData> new_cloud_data_buff_;
  };
}

#endif