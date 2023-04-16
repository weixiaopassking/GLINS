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
// sub
#include "../../subscriber/cloud_subscriber.hpp"
#include "../../subscriber/odometry_subscriber.hpp"
#include "../../subscriber/key_frames_subscriber.hpp"
#include "../../subscriber/key_frame_subscriber.hpp"
// pub
#include "../../publisher/odometry_publisher.hpp"
#include "../../publisher/cloud_publisher.hpp"
//数据类型
#include "../../sensor_data/pose_data.hpp"
#include "../../sensor_data/key_frame.hpp"
//viewer
#include "./viewer.hpp"
//c++
#include <deque>

namespace multisensor_localization
{
    class ViewerFlow
    {
    public:
        ViewerFlow(ros::NodeHandle &nh);

        bool Run();
        bool SaveMap();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();


    private:
        /*订阅器*/
        std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
        std::shared_ptr<Viewer> viewer_ptr_;

        std::deque<KeyFrame> optimized_key_frames_;
        
    };

} // namespace multisensor_localization

#endif