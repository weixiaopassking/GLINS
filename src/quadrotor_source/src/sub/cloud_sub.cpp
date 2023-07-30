#include "./cloud_sub.hpp"

namespace rossub_ns
{
CloudSub::CloudSub(ros::NodeHandle &node_handle, const std::string topic_name, const size_t buffer_size)
{
    _sub = _nh.subscribe(topic_name, buffer_size, &CloudSub::MsgCallback, this);
}

void CloudSub::ParseData(std::deque<common_ns::CloudType> &cloud_ptr_que)
{
}

void CloudSub::MsgCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    std::cout << "has receiver" << std::endl;
}
} //    namespace rossub_ns