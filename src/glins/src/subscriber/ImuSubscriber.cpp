/*
 * @Description:imu  subscriber
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-26
 */

// relevent
#include "../../include/subscriber/ImuSubscriber.hpp"


namespace glins
{
    /**
     * @brief  config ros topic
     * @note
     * @todo
     **/
    ImuSubscriber::ImuSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe<sensor_msgs::Imu>(topic_name, queue_size, &ImuSubscriber::MsgCallbcak, this,ros::TransportHints().tcpNoDelay());
    }
    /**
     * @brief  callback function
     * @note push the data into buff
     * @todo
     **/
    void ImuSubscriber::MsgCallbcak(const sensor_msgs::ImuConstPtr &msg)
    {
        ImuDataType imu_data;

        imu_data.time_stamp = msg->header.stamp.toSec();

        imu_data.linear_acceleration.x = msg->linear_acceleration.x;
        imu_data.linear_acceleration.y = msg->linear_acceleration.y;
        imu_data.linear_acceleration.z = msg->linear_acceleration.z;

        imu_data.angular_velocity.x = msg->angular_velocity.x;
        imu_data.angular_velocity.y = msg->angular_velocity.y;
        imu_data.angular_velocity.z = msg->angular_velocity.z;

        imu_data.orientation.x = msg->orientation.x;
        imu_data.orientation.y = msg->orientation.y;
        imu_data.orientation.z = msg->orientation.z;
        imu_data.orientation.w = msg->orientation.w;

        data_buffer_.push_back(imu_data);
    }

    /**
     * @brief  read data from buff
     * @note read and clear the buff
     * @todo
     **/
    void ImuSubscriber::ParseData(std::deque<ImuDataType> &data_queue)
    {
        if (data_buffer_.size() > 0)
        {
            data_queue.insert(data_queue.end(), data_buffer_.begin(), data_buffer_.end());
            data_buffer_.clear();
        }
    }

} // namespace glins