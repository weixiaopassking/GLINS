/*
 * @Description: 多关键帧发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

// relevent
#include "../../include/publisher/loop_pose_publisher.hpp"
// ros
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace multisensor_localization
{
    /**
     * @brief 回环检测发布初始化
     * @note
     * @todo
     **/
    LoopPosePublisher::LoopPosePublisher(ros::NodeHandle &nh,
                                         std::string topic_name,
                                         std::string frame_id,
                                         int buff_size)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
    }

    /**
     * @brief 是否被订阅状态
     * @note
     * @todo
     **/
    bool LoopPosePublisher::HasSubscriber()
    {
        return publisher_.getNumSubscribers() != 0;
    }

    /**
     * @brief 回环位姿发布
     * @note
     * @todo
     **/
    void LoopPosePublisher::Publish(LoopPose &loop_pose)
    {
        geometry_msgs::PoseWithCovarianceStamped pose_stamped;

        /*拷贝时间戳*/
        ros::Time ros_time((float)loop_pose.time_stamp_);
        pose_stamped.header.stamp = ros_time;
        /*拷贝frame id*/
        pose_stamped.header.frame_id = frame_id_;
        /*拷贝平移量*/
        pose_stamped.pose.pose.position.x = loop_pose.pose(0, 3);
        pose_stamped.pose.pose.position.y = loop_pose.pose(1, 3);
        pose_stamped.pose.pose.position.z = loop_pose.pose(2, 3);
        /*拷贝旋转量*/
        Eigen::Quaternionf q = loop_pose.GetQuaternion();
        pose_stamped.pose.pose.orientation.x = q.x();
        pose_stamped.pose.pose.orientation.y = q.y();
        pose_stamped.pose.pose.orientation.z = q.z();
        /*用协方差表示序号无奈之举*/
        pose_stamped.pose.covariance[0] = (double)loop_pose.index0_;
        pose_stamped.pose.covariance[1] = (double)loop_pose.index1_;

        /*发布*/
        publisher_.publish(pose_stamped);
    }

} // namespace multisensor_localization