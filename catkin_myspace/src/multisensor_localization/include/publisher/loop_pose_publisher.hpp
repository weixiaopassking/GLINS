/*
 * @Description: 回环检测发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-22
 */

#ifndef LOOP_POSE_PUBLISHER_HPP_
#define LOOP_POSE_PUBLISHER_HPP_

// ros
#include <ros/ros.h>
// c++
#include <deque>
// sensor_data
#include "../sensor_data/loop_pose.hpp"

namespace multisensor_localization
{

    class LoopPosePublisher
    {
    public:
        LoopPosePublisher(ros::NodeHandle &nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);

        LoopPosePublisher() = default;

        void Publish(LoopPose &loop_pose);

        bool HasSubscriber();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";

    }; // class LoopPosePublisher

} // namespace multisensor_localization

#endif