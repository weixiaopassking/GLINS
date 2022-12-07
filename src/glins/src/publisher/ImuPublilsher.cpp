/*
 * @Description: publisher imu data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 * @Note:
 */

// relevent
#include "../../include/publisher/ImuPublisher.hpp"

namespace glins
{
    /**
     * @brief  config topic for publishing
     * @note
     * @todo
     **/
    ImuPublisher::ImuPublisher(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size, const std::string frame_id)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, queue_size);
    }

    /**
     * @brief function overloading1
     * @note internal time stamp
     * @todo
     **/
    void ImuPublisher::Publish(ImuDataType &imu_input)
    {
        ros::Time time = ros::Time::now();
        PublishData(imu_input, time);
    }

    /**
     * @brief function overloading2
     * @note external time stamp
     * @todo
     **/
    void ImuPublisher::Publish(ImuDataType &imu_input, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(imu_input, ros_time);
    }

    /**
     * @brief publish data
     * @note  internal time stamp
     * @todo
     **/
    void ImuPublisher::PublishData(ImuDataType &imu_input, ros::Time time)
    {

        /*header*/
        imu_.header.frame_id=frame_id_;
        imu_.header.stamp=time;

        /*orientation*/
        imu_.orientation.w = imu_input.orientation.w;
        imu_.orientation.x = imu_input.orientation.x;
        imu_.orientation.y = imu_input.orientation.y;
        imu_.orientation.z = imu_input.orientation.z;

        /*linear acceleration*/
        imu_.linear_acceleration.x = imu_input.linear_acceleration.x;
        imu_.linear_acceleration.y = imu_input.linear_acceleration.y;
        imu_.linear_acceleration.z = imu_input.linear_acceleration.z;

        /*angular velocity*/
        imu_.angular_velocity.x = imu_input.angular_velocity.x;
        imu_.angular_velocity.y = imu_input.angular_velocity.y;
        imu_.angular_velocity.z = imu_input.angular_velocity.z;
    }

    /**
     * @brief check
     * @note external time stamp
     * @todo
     **/
    bool ImuPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
} // namespace glins
