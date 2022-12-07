/*
 * @Description: publisher imu data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-07
 * @Note:
 */

#ifndef IMU_PUBLISHER_HPP_
#define IMU_PUBLISHER_HPP_

// ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// data type
#include "../sensor_data/ImuDataType.hpp"

namespace glins
{
    class ImuPublisher
    {
    public:
        ImuPublisher(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size, const std::string frame_id);
        ImuPublisher() = default;

        void Publish(ImuDataType &imu_input, double time);
        void Publish(ImuDataType &imu_input);

        bool HasSubscribers();

    private:
        void PublishData(ImuDataType &imu_input, ros::Time time);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::Imu imu_;
        std::string frame_id_;
    }; // class EnuPublisher

} // namespace glins

#endif