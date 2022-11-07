/*
 * @Description: 多关键帧发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

// relevent
#include "../../include/publisher/odometry_publisher.hpp"

namespace multisensor_localization
{

    /**
     * @brief 里程计发布初始化
     * @note
     * @todo
     **/
    OdometryPublisher::OdometryPublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         int buff_size) : nh_(nh)
    {

        publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        odometry_.header.frame_id = base_frame_id;
        odometry_.child_frame_id = child_frame_id;
    }

    /**
     * @brief 里程计发布信息
     * @note 重载(取外参作时间戳)
     * @todo
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(transform_matrix, ros_time);
    }

    /**
     * @brief 里程计发布信息
     * @note 重载(取发布时间做时间戳)
     * @todo
     **/
    void OdometryPublisher::Publish(const Eigen::Matrix4f &transform_matrix)
    {
        PublishData(transform_matrix, ros::Time::now());
    }

    /**
     * @brief 里程数据发布
     * @note
     * @todo
     **/
    void OdometryPublisher::PublishData(const Eigen::Matrix4f &transform_matrix, ros::Time time)
    {
        /*填充header*/
        odometry_.header.stamp = time;

        /*填充三轴位姿数据*/
        odometry_.pose.pose.position.x = transform_matrix(0, 3);
        odometry_.pose.pose.position.y = transform_matrix(1, 3);
        odometry_.pose.pose.position.z = transform_matrix(2, 3);

        /*填充旋转数据*/
        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry_.pose.pose.orientation.x = q.x();
        odometry_.pose.pose.orientation.y = q.y();
        odometry_.pose.pose.orientation.z = q.z();
        odometry_.pose.pose.orientation.w = q.w();
        /*发布*/
        publisher_.publish(odometry_);
    }

    /**
     * @brief 是否被订阅
     * @note
     * @todo
     **/
    bool OdometryPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
}