#include "odom_pub.hpp"

namespace pub_ns
{

OdomPub::OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id, const std::string child_frame_id,
            const int buffer_size)
{
    _nh= nh;
    _pub = _nh.advertise<nav_msgs::Odometry>(topic_name, buffer_size);
    _odom.header.frame_id = base_frame_id;
    _odom.child_frame_id = child_frame_id;
}

void OdomPub::Pub(const data_ns::Mat4d  &transform_matrix, double time)
{
    ros::Time ros_time((float)time);
    PubData(transform_matrix, ros_time);
}

void OdomPub::Pub(const data_ns::Mat4d &transform_matrix)
{
    PubData(transform_matrix, ros::Time::now());
}


void OdomPub::PubData(const data_ns::Mat4d  &transform_matrix, ros::Time time)
{

    _odom.header.stamp = time;


    _odom.pose.pose.position.x = transform_matrix(0, 3);
    _odom.pose.pose.position.y = transform_matrix(1, 3);
    _odom.pose.pose.position.z = transform_matrix(2, 3);

    data_ns::Quatd q;
    q = transform_matrix.block<3, 3>(0, 0);
    _odom.pose.pose.orientation.x = q.x();
    _odom.pose.pose.orientation.y = q.y();
    _odom.pose.pose.orientation.z = q.z();
    _odom.pose.pose.orientation.w = q.w();
    
    _pub.publish(_odom);
}


bool OdomPub::HasSubscribers()
{
    return _pub.getNumSubscribers() != 0;
}
} // namespace pub_ns