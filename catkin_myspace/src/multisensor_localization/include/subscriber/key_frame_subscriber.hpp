/*
 * @Description:关键帧接收
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

#ifndef KEY_FRAME_SUBSCRIBER_HPP_
#define KEY_FRAME_SUBSCRIBER_HPP_

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//c++
#include <deque>
//snesor data
#include "../sensor_data/key_frame.hpp"

namespace multisensor_localization
{
    class KeyFrameSubscriber
    {

        KeyFrameSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        KeyFrameSubscriber() = default;

        void Parse(std::deque<KeyFrame>key_frame_buff);

        private:
        void MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msgs_ptr);

        private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<KeyFrame> new_key_frame_buff_;

    }; // class KeyFrameSubscriber

} // namespace multisensor_localization

#endif