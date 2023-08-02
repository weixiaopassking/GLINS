#ifndef _ODOM_PUB_HPP
#define _ODOM_PUB_HPP

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include "../data/geometry_data.hpp"

namespace pub_ns
{
class OdomPub
{
  public:
    OdomPub() = delete;
    OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id, const std::string child_frame_id,
            const int buffer_size);

    void Pub(const data_ns::Mat4f &transform_matrix, double time);
    void Pub(const data_ns::Mat4f &transform_matrix);

    bool HasSubscribers();

  private:
    void PubData(const data_ns::Mat4f &transform_matrix, ros::Time time);

  private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    nav_msgs::Odometry _odom;
};
} // namespace pub_ns
#endif //ODOM_PUB_HPP_