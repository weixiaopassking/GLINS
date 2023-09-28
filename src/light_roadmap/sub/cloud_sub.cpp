/*
 * @Description: cloud subscriber
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-09-28
 */

#include "cloud_sub.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace sub
{

CloudSub::CloudSub()
{
}

CloudSub()::~CloudSub()
{
}

CloudSub::CloudSub(const ros::NodeHandle &nh, const std::string &topic_name, const size_t buffer_size)
{
    _nh = nh;
    _sub = _nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    msg::CloudData cloud_data;
    cloud_data.timestamp_ns = cloud_msg_ptr->header.stamp.toSec() * 10e9;
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr)); //! risk
    _cloud_data_que.push_back(cloud_data);
}

void CloudSub::ParseData(std::deque<msg::CloudData> &cloud_data_queue)
{
    if (_cloud_data_que.size() > 0)
    {
        cloud_data_queue.insert(cloud_data_queue.end(), _cloud_data_que.begin(), _cloud_data_que.end());
        _cloud_data_que.clear();
    }
}

} // namespace sub