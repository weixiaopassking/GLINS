/*
 * @Description: publisher ENU origin
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 * @Note: Using the WGS 84 reference ellipsoid
 */

#ifndef ENU_PUBLISHER_HPP_
#define ENU_PUBLISHER_HPP_

// ros
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
// data type
#include "../sensor_data/GnssDataType.hpp"

namespace glins
{
    class EnuPublisher
    {
    public:
        EnuPublisher(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size, const std::string frame_id);
        EnuPublisher() = default;

        void Publish(GnssFixDataType &enu_input);
        bool HasSubscribered();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::NavSatFix origin_;
        std::string frame_id_;
    };//class EnuPublisher

} // namespace glins

#endif