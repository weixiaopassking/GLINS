/*
 * @Description: imu subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

// relevent
#include "imu_sub.hpp"

namespace sub_ns
{
/**
 * @brief    imu sub init
 * @param
 * @note
 **/
IMUSub::IMUSub(ros::NodeHandle &nh, const std::string& topic_name, const size_t buffer_size)
{
    _nh=nh;
    _sub = _nh.subscribe(topic_name, buffer_size, &IMUSub::MsgCallback, this);
}

/**
 * @brief msg callback
 * @note
 * @todo
 **/
void IMUSub::MsgCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr)
{
    data_ns::IMUData imu_data;
    imu_data._time_stamp = imu_msg_ptr->header.stamp.toSec();

    imu_data._linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
    imu_data._linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
    imu_data._linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

    imu_data._angular_velocity.x = imu_msg_ptr->angular_velocity.x;
    imu_data._angular_velocity.y = imu_msg_ptr->angular_velocity.y;
    imu_data._angular_velocity.z = imu_msg_ptr->angular_velocity.z;

    imu_data._orientation.x = imu_msg_ptr->orientation.x;
    imu_data._orientation.y = imu_msg_ptr->orientation.y;
    imu_data._orientation.z = imu_msg_ptr->orientation.z;
    imu_data._orientation.w = imu_msg_ptr->orientation.w;

    _imu_data_deq.push_back(imu_data);
}

/**
 * @brief read buffer
 * @note
 * @todo
 **/
void IMUSub::ParseData(std::deque<data_ns::IMUData> &imu_data_deq)
{
    if (_imu_data_deq.size() > 0)
    {
        imu_data_deq.insert(imu_data_deq.end(), _imu_data_deq.begin(), _imu_data_deq.end());
        _imu_data_deq.clear();
    }
}

} // namespace sub_ms