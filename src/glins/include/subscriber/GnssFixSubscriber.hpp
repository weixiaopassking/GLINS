/*
 * @Description: receive gnss fixed from gnss device
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-17
 */

#ifndef GNSS_FIX_SUBSCRIBER_HPP_
#define GNSS_FIX_SUBSCRIBER_HPP_

// c++
#include <deque>
// ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h> 
// sensor data type
#include "../../include/sensor_data/GnssDataType.hpp"

namespace glins
{
    class GnssFixSubscriber
    {
    public:
        GnssFixSubscriber() = default;
        GnssFixSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size);
        void ParseData(std::deque<GnssFixData> &data_deque);

    private:
        void MsgCallbcak(const sensor_msgs::NavSatFixConstPtr &msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GnssFixData> data_buff_;
    }; // class glins

} // namespace glins

#endif