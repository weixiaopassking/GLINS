/*
 * @Description: 速度数据订阅
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-15
 */

#ifndef VELOCITY_SUBSCRIBER_HPP_
#define VELOCITY_SUBSCRIBER_HPP_

//c++
#include <deque>
//ros
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
//速度自定义数据
#include "../sensor_data/velocity_data.hpp"

namespace multisensor_localization
{
  class VelocitySubscriber
  {
  public:
    VelocitySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData> &velocity_data_buff);

  private:
    void MsgCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_buff_;
  };
}
#endif