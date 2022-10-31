/*
 * @Description: 前端任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-31
 */

#ifndef VIEWER_FLOW_HPP_
#define VIEWER_FLOW_HPP_

// ros
#include <ros/ros.h>
//sub
#include "../../subscriber/cloud_subscriber.hpp"
#include "../../subscriber/odometry_subscriber.hpp"
#include "../../subscriber/key_frames_subscriber.hpp"
#include "../../subscriber/key_frame_subscriber.hpp"
//pub
#include "../../publisher/odometry_publisher.hpp"
#include "../../publisher/cloud_publisher.hpp"

namespace multisensor_localization
{
    class ViewerFlow
    {
        ViewerFlow(ros::NodeHandle &nh);

        bool Run();
        bool SaveMap();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool PublishGlobalData();
        bool PublishLocalData();

    private:
        
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;

        std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;

        //std::shared_ptr<Viewer>
    };

} // namespace multisensor_localization

#endif