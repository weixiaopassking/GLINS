/*
 * @Description: imu subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#ifndef _IMU_SUB_HPP
#define _IMU_SUB_HPP

//data 
#include "../data/imu_data.hpp"
//system
#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace sub_ns
{
class IMUSub
{
  public:
    IMUSub() = delete; //must set node handle
    IMUSub(ros::NodeHandle &nh, const std::string& topic_name, const size_t buffer_size);

    void ParseData(std::deque<data_ns::IMUData> &imu_data_deq);

  private:
    void MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    std::deque<data_ns::IMUData> _imu_data_deq;

}; // class IMUSub
} // namespace sub_ns

#endif //_IMU_SUB_HPP