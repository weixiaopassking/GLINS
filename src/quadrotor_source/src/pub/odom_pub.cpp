#include "odom_pub.hpp"

namespace rospub_ns
{

OdomPub::OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id,
                 const std::string child_frame_id, const int buffer_size)
{
    _nh= nh;
    _pub = _nh.advertise<nav_msgs::Odometry>(topic_name, buffer_size);
    _odom.header.frame_id = base_frame_id;
    _odom.child_frame_id = child_frame_id;
}


void OdomPub::Publish(const Eigen::Matrix4f &transform_matrix, double time)
{
    ros::Time ros_time((float)time);
    PublishData(transform_matrix, ros_time);
}


void OdomPub::Publish(const Eigen::Matrix4f &transform_matrix)
{
    PublishData(transform_matrix, ros::Time::now());
}

/**
 * @brief 里程数据发布
 * @note
 * @todo
 **/
void OdomPub::PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time)
{
    /*填充header*/
    _odom.header.stamp = time;

    /*填充三轴位姿数据*/
    _odom.pose.pose.position.x = transform_matrix(0, 3);
    _odom.pose.pose.position.y = transform_matrix(1, 3);
    _odom.pose.pose.position.z = transform_matrix(2, 3);

    /*填充旋转数据*/
    Eigen::Quaternionf q;
    q = transform_matrix.block<3, 3>(0, 0);
    _odom.pose.pose.orientation.x = q.x();
    _odom.pose.pose.orientation.y = q.y();
    _odom.pose.pose.orientation.z = q.z();
    _odom.pose.pose.orientation.w = q.w();
    /*发布*/
    _pub.publish(_odom);
}


bool OdomPub::HasSubscribers()
{
    return _pub.getNumSubscribers() != 0;
}
} // namespace rospub_ns