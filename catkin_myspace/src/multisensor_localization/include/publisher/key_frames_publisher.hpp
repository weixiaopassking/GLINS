/*
 * @Description: 多关键帧发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef KEY_FRAMES_HPP_
#define KEY_FRAMES_HPP_

//ros
#include <ros/ros.h>
//c++
#include <string>
#include <deque>
//自定义关键帧数据
#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{
  class KeyFramesPublisher
  {
  public:
    KeyFramesPublisher(ros::NodeHandle &nh,
                       std::string topic_name,
                       std::string frame_id,
                       int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame> &key_frames_buff);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
  };

} // namespace name

#endif