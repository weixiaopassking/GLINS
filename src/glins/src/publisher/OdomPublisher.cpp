/*
 * @Description: odom data publish
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-24
 * @Note:
 */

//relevent
#include "../../include/publisher/OdomPublisher.hpp"

namespace glins
{

    /**
     * @brief odom publish init
     * @note
     * @todo
     **/
    OdomPublisher::OdomPublisher(ros::NodeHandle &nh,
                     const  std::string topic_name,
                      const std::string base_frame_id,
                      const std::string child_frame_id,
                      const size_t queue_size) : nh_(nh)
    {
        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, queue_size);
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    /**
     * @brief function overloading1
     * @note external time stamp 
     * @todo
     **/
    void OdomPublisher::Publish(const Eigen::Matrix4f &transform_matrix, double time_stamp)
    {
        ros::Time ros_time((float)time_stamp);
        PublishData(transform_matrix, ros_time);
    }

    /**
     * @brief function overloading2
     * @note  internal time stamp 
     * @todo
     **/
    void OdomPublisher::Publish(const Eigen::Matrix4f &transform_matrix)
    {
        PublishData(transform_matrix, ros::Time::now());
    }

   /**
     * @brief publish data
     * @note  internal time stamp 
     * @todo
     **/
    void OdomPublisher::PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time_stamp)
    {
        /*header*/
        odometry_.header.stamp = time_stamp;

        /*translation*/
        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        /*rotation*/
        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();
        /*publish*/
        publisher_.publish(odometry_);
    }

    /**
     * @brief  subscriber status
     * @note
     * @todo
     **/
    bool OdomPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
}