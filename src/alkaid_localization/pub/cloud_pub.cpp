#include "cloud_pub.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace pub_ns
{

CloudPub::CloudPub(ros::NodeHandle &nh, const std::string topic_name, const std::string frame_id,
                               const size_t buffer_size)

{
    _nh = nh;
    _frame_id = frame_id;
    _pub = _nh.advertise<sensor_msgs::PointCloud2>(topic_name, buffer_size);
}

void CloudPub::Pub(data_ns::CloudData::CLOUD_PTR &cloud_ptr, double time)
{
    ros::Time ros_time((float)time);
    PubData(cloud_ptr, ros_time);
}

void CloudPub::Pub(data_ns::CloudData::CLOUD_PTR &cloud_ptr)
{
    ros::Time time = ros::Time::now();
    PubData(cloud_ptr, time);
}

void CloudPub::PubData(data_ns::CloudData::CLOUD_PTR &cloud_ptr, ros::Time time)
{

    sensor_msgs::PointCloud2Ptr cloud_ros_ptr(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr, *cloud_ros_ptr);

    cloud_ros_ptr->header.stamp = time;
    cloud_ros_ptr->header.frame_id = _frame_id;
    _pub.publish(*cloud_ros_ptr);
}

bool CloudPub::HasSubscribered()
{
    return _pub.getNumSubscribers() != 0;
}

} // namespace pub_ns