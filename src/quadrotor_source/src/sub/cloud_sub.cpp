/**
*****************************************************************************
*  Copyright (C), 2023-2026,wengang.niu
*  @file cloud_sub.cpp
*  @brief  点云信息接收
*  @author  wengang.niu
*  @date    2023/7/31
*  @version v0.2
*  @ref github.com/Little-Potato-1990/localization_in_auto_driving
*  @note
****************************************************************************
*/

#include "./cloud_sub.hpp"

namespace rossub_ns
{

/**
 * @brief  初始化接收
 * @param
 * @return
 * @note
 **/
CloudSub::CloudSub(ros::NodeHandle &node_handle, const std::string topic_name, const size_t buffer_size)
{
    _sub = _nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

/**
 * @brief  获取消息队列
 * @param
 * @return
 * @note
 **/
void CloudSub::ParseData(std::deque<common_ns::CloudType> &cloud_msg_que)
{
    if (_cloud_msg_que.size() > 0)
    {
        cloud_msg_que.insert(cloud_msg_que.end(), _cloud_msg_que.begin(), _cloud_msg_que.end());
        _cloud_msg_que.clear();
    }

}

/**
 * @brief  消息回调
 * @param
 * @return
 * @note
 **/
void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    common_ns::CloudType cloud_msg_temp;
    cloud_msg_temp._timestamp = cloud_msg->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg, *(cloud_msg_temp._cloud_ptr));

    _cloud_msg_que.push_back(cloud_msg_temp);
    std::cout << "received cloud number is" << cloud_msg_temp._cloud_ptr->points.size() << std::endl;
}
} //    namespace rossub_ns