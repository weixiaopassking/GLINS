/*
 * @Description: 单个关键帧发布
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef KEY_FRAME_PUBLISHER_HPP_
#define KEY_FRAME_PUBLISHER_HPP_

//ros
#include <ros/ros.h>
//c++
#include <deque>
//自定义关键帧数据结构
#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{

    class KeyFramePublisher
    {
        public:
        KeyFramePublisher(ros::NodeHandle &nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        KeyFramePublisher() = default;
        void Publish(KeyFrame &key_frame);
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };

} // namespace name

#endif