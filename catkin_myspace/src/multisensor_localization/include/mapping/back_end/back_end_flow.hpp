/*
 * @Description: 后端任务管理器
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-19
 */

#ifndef BACK_END_FLOW_HPP_
#define BACK_END_FLOW_HPP_

// ros
#include <ros/ros.h>
//话题接收
#include "../../../include/subscriber/cloud_subscriber.hpp"
#include "../../../include/subscriber/gnss_subscriber.hpp"
#include "../../../include/subscriber/odometry_subscriber.hpp"
//话题发布
#include "../../../include/publisher/odometry_publisher.hpp"
#include "../../../include/publisher/key_frames_publisher.hpp"
#include "../../../include/publisher/key_frame_publisher.hpp"
//后端算法
#include "./back_end.hpp"

namespace multisensor_localization
{
    class BackEndFlow
    {
    public:
        BackEndFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateBackEnd();
        bool PublishData();

    private:
        std::shared_ptr<BackEnd> back_end_ptr_;

        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_odom_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

        std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
        std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_odom_data_buff_;
        std::deque<PoseData> laser_odom_data_buff_;

        PoseData current_gnss_pose_data_;
        PoseData current_laser_odom_data_;
        CloudData current_cloud_data_;
    };
}

#endif
