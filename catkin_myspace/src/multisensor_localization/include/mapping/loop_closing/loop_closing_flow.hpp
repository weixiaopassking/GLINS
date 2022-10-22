/*
 * @Description: 回环检测任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

#ifndef LOOP_CLOSING_FLOW_HPP_
#define LOOP_CLOSING_FLOW_HPP_

//relevent
#include "../../../include/mapping/loop_closing/loop_closing.hpp"
// c++
#include <deque>
// ros
#include <ros/ros.h>
// sensor data
#include "../../../include/sensor_data/key_frame.hpp"
// subscriber
#include "../../../include/subscriber/key_frame_subscriber.hpp"
// publisher
#include "../../../include/publisher/loop_pose_publisher.hpp"


namespace multisensor_localization
{
    class LoopClosinigFlow
    {
    public:
        LoopClosinigFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool PublishData();

    private:
        // subscriber
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
        // publisher
        std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
        //loop closing 
        std::shared_ptr<LoopClosing> loop_closing_ptr_;

        //数据列
        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> key_gnss_buff_;
        //当前数据
        KeyFrame current_key_frame_;
        KeyFrame current_key_gnss_;

    }; // class LoopClosinigFlow

} // namespace multisensor_localization

#endif
