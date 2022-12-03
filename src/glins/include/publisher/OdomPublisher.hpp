/*
 * @Description: publisher odom
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-24
 * @Note:
 */

#ifndef ODOM_PUBLISHER_HPP_
#define ODOM_PUBLISHER_HPP_

// ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// c++
#include <string>
//thirdpart lib
#include <Eigen/Dense>

namespace glins
{
  class OdomPublisher
  {
  public:
    OdomPublisher(ros::NodeHandle &nh,
                     const  std::string topic_name,
                      const std::string base_frame_id,
                      const std::string child_frame_id,
                      const size_t queue_size);
    OdomPublisher() = default;

    void Publish(const Eigen::Matrix4f &transform_matrix, const double time_stamp);
    void Publish(const Eigen::Matrix4f &transform_matrix);

    bool HasSubscribers();

  private:
    void PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time_stamp);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
  };
}
#endif