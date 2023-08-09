/*
 * @Description: odom pub
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#include "odom_pub.hpp"

namespace pub_ns
{

/**
 * @brief    cloud  pub init
 * @param
 * @param
 * @note
 **/
OdomPub::OdomPub(ros::NodeHandle &nh, const std::string& topic_name, const std::string& base_frame_id,
                 const std::string &child_frame_id, const int buffer_size)
{
    _nh = nh;
    _pub = _nh.advertise<nav_msgs::Odometry>(topic_name, buffer_size);
    _odom.header.frame_id = base_frame_id;
    _odom.child_frame_id = child_frame_id;
}

/**
 * @brief    cloud  pub init
 * @param
 * @param
 * @note
 **/
void OdomPub::Pub(const data_ns::Mat4f &odom_matrix, double time_stamp)
{
    ros::Time ros_time_stamp((float)time_stamp);
    PubData(odom_matrix, ros_time_stamp);
}

/**
 * @brief    cloud  pub overload1
 * @param
 * @param
 * @note
 **/
void OdomPub::Pub(const data_ns::Mat4f &odom_matrix)
{
    PubData(odom_matrix, ros::Time::now());
}

/**
 * @brief    cloud  pub overload2
 * @param
 * @param
 * @note
 **/
void OdomPub::PubData(const data_ns::Mat4f &odom_matrix, ros::Time time_stamp)
{

    _odom.header.stamp = time_stamp;

    _odom.pose.pose.position.x = odom_matrix(0, 3);
    _odom.pose.pose.position.y = odom_matrix(1, 3);
    _odom.pose.pose.position.z = odom_matrix(2, 3);

    data_ns::Quatf q;
    q = odom_matrix.block<3,3>(0,0);
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