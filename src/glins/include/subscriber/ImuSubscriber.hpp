/*
 * @Description: receive imu data from imu device
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-26
 */

#ifndef IMU_SUBSCRIBER_HPP_
#define IMU_SUBSCRIBER_HPP_

// c++
#include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// sensor data type
#include "../../include/sensor_data/ImuDataType.hpp"

namespace glins
{
    class ImuSubscriber
    {
    public:
        ImuSubscriber() = default;
        ImuSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size);
        void ParseData(std::deque<ImuDataType> &data_queue);

    private:
        void MsgCallbcak(const sensor_msgs::ImuConstPtr &msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<ImuDataType> data_buffer_;
    };

} // namesapce glins

#endif
