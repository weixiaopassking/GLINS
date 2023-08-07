/*
 * @Description: cloud subscriber
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#include "cloud_sub.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace sub_ns
{

/**
 * @brief     cloud subscriber
 * @param 
 * @param  
 * @note
 **/
CloudSub::CloudSub(ros::NodeHandle &nh, const std::string &topic_name, const size_t buffer_size)
{
    _nh = nh;
    _sub = _nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

/**
 * @brief     cloud subscriber callback 
 * @param
 * @note
 **/
void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    data_ns::CloudData cloud_data;
    cloud_data._time_stamp = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data._cloud_ptr));
    _cloud_data_que.push_back(cloud_data);
}

/**
 * @brief     parese data from buffer
 * @param
 * @note
 **/
void CloudSub::ParseData(std::deque<data_ns::CloudData> &cloud_data_queue)
{
    if (_cloud_data_que.size() > 0)
    {
        cloud_data_queue.insert(cloud_data_queue.end(), _cloud_data_que.begin(), _cloud_data_que.end());
        _cloud_data_que.clear();
    }
}

} // namespace sub_ns