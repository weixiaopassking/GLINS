/*
 * @Description: publish a single key frame
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

//relevent
#include "../../include/publisher/key_frame_publisher.hpp"
//ros
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//eigen
#include <Eigen/Dense>

namespace multisensor_localization
{

    /**
     * @brief 关键帧发布器初始化
     * @note
     * @todo
     **/
    KeyFramePublisher::KeyFramePublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         std::string frame_id,
                                         int buff_size)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
    }


    /**
     * @brief 关键帧发布
     * @note
     * @todo
     **/
    void KeyFramePublisher::Publish(KeyFrame &key_frame)
    {
        geometry_msgs::PoseWithCovarianceStamped pose_stamped;

        ros::Time ros_time((float)key_frame.time_stamp_);
        /*填充header*/
        pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = frame_id_;
        /*填充三轴位置数据*/
        pose_stamped.pose.pose.position.x = key_frame.pose_(0, 3);
        pose_stamped.pose.pose.position.y = key_frame.pose_(1, 3);
        pose_stamped.pose.pose.position.z = key_frame.pose_(2, 3);
     /*填充旋转数据*/
        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_stamped.pose.pose.orientation.x = q.x();
        pose_stamped.pose.pose.orientation.y = q.y();
        pose_stamped.pose.pose.orientation.z= q.z();
        pose_stamped.pose.pose.orientation.w = q.w();
        /*序号放入协方差当中*/
        pose_stamped.pose.covariance[0]=key_frame.index_;
        /*发布*/
        publisher_.publish(pose_stamped);
    }

    /**
     * @brief 话题是否被订阅
     * @note
     * @todo
     **/
    bool KeyFramePublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }

}; // multisensor_localization
