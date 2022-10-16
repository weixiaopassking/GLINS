/*
 * @Description: 里程计发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef ODOMETRY_PUBLISHER_HPP_
#define ODOMETRY_PUBLISHER_HPP_

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//c++
#include <string>
//eigen
#include <Eigen/Dense>


namespace multisensor_localization
{
  class OdometryPublisher
  {
  public:
    OdometryPublisher(ros::NodeHandle &nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    OdometryPublisher() = default;

    void Publish(const Eigen::Matrix4f &transform_matrix, double time);
    void Publish(const Eigen::Matrix4f &transform_matrix);

    bool HasSubscribers();

  private:
     void PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
  };
}
#endif