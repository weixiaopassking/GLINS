/*
 * @Description:关键帧接收
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

// relevent
#include "../../include/subscriber/key_frames_subscriber.hpp"

namespace multisensor_localization
{
    /**
     * @brief 关键帧队列 发送初始化
     * @note
     * @todo
     **/
    KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFramesSubscriber::MsgCallback, this);
    }
    /**
     * @brief 关键帧读取数据 但只能会缓存一次
     * @note
     * @todo
     **/
    void KeyFramesSubscriber::ParseData(std::deque<KeyFrame> &key_frames_buff)
    {
        if (new_key_frames_buff_.size() > 0)
        {
            key_frames_buff = new_key_frames_buff_;
            new_key_frames_buff_.clear();
        }
    }

    /**
     * @brief 关键帧队列回调函数
     * @note
     * @todo
     **/
    void KeyFramesSubscriber::MsgCallback(const nav_msgs::Path::ConstPtr &key_frames_msg_ptr)
    {
        new_key_frames_buff_.clear();

        for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); i++)
        {
            KeyFrame key_frame;
            key_frame.time_stamp_ = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
            key_frame.index_ = (unsigned int)i;
            /*位移拷贝*/
            key_frame.pose_(0, 3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
            key_frame.pose_(1, 3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
            key_frame.pose_(2, 3) = key_frames_msg_ptr->poses.at(i).pose.position.z;
            /*旋转拷贝*/
            Eigen::Quaternionf q;
            q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
            q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
            q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
            q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;
            key_frame.pose_.block<3, 3>(0, 0) = q.matrix();
            /*压入队列*/
            new_key_frames_buff_.push_back(key_frame);
        }
    }

} // namespace multisensor_localization
