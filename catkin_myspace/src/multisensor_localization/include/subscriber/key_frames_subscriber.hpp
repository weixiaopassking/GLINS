/*
 * @Description:关键帧队列接收
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

#ifndef KEY_FRAMES_SUBSCRIBER_HPP_
#define KEY_FRAMES_SUBSCRIBER_HPP_

// ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>
// snesor data
#include "../sensor_data/key_frame.hpp"
// c++
#include <deque>

namespace multisensor_localization
{

    class KeyFramesSubscriber
    {

    public:
        KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        KeyFramesSubscriber() = default;
        void ParseData(std::deque<KeyFrame> &deque_key_frames);

    private:
        void MsgCallback(const nav_msgs::Path::ConstPtr &key_frames_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<KeyFrame> new_key_frames_buff_;

    }; //    class KeyFramesSubscriber

} // namespace  multisensor_localization

#endif
