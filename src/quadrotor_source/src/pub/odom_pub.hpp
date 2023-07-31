#ifndef ODOM_PUB_HPP_
#define ODOM_PUB_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>

namespace rospub_ns
{
class OdomPub
{
  public:
    OdomPub() = delete;
    OdomPub(ros::NodeHandle &nh, const std::string topic_name, const std::string base_frame_id, const std::string child_frame_id,
            const int buffer_size=1);

    void Publish(const Eigen::Matrix4f &transform_matrix, double time);
    void Publish(const Eigen::Matrix4f &transform_matrix);

    bool HasSubscribers();

  private:
    void PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time);

  private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    nav_msgs::Odometry _odom;
};
} // namespace rospub_ns
#endif //ODOM_PUB_HPP_