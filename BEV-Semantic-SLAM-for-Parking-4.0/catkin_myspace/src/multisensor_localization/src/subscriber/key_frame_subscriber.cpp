/*
 * @Description:关键帧发送
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */


// relevent
#include "../../include/subscriber/key_frame_subscriber.hpp"

namespace multisensor_localization
{
    /**
     * @brief 关键帧发送初始化
     * @note
     * @todo
     **/
    KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::MsgCallback, this);
    }

    /**
     * @brief 数据读取
     * @note
     * @todo
     **/
    void KeyFrameSubscriber::ParseData(std::deque<KeyFrame> key_frame_buff)
    {
        if (new_key_frame_buff_.size() > 0)
        {
            key_frame_buff.insert(key_frame_buff.end(), new_key_frame_buff_.begin(), new_key_frame_buff_.end());
        }
        new_key_frame_buff_.clear();
    }

    /**
     * @brief 回调函数
     * @note
     * @todo
     **/
    void KeyFrameSubscriber::MsgCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &key_frame_msgs_ptr)
    {
        KeyFrame key_frame;
        /*拷贝时间戳*/
        key_frame.time_stamp_ = key_frame_msgs_ptr->header.stamp.toSec();
        /*拷贝序号 无奈只能用协方差传递序号了*/
        key_frame.index_ = (unsigned int)key_frame_msgs_ptr->pose.covariance[0];
        /*拷贝平移量*/
        key_frame.pose_(0, 3) = key_frame_msgs_ptr->pose.pose.position.x;
        key_frame.pose_(1, 3) = key_frame_msgs_ptr->pose.pose.position.y;
        key_frame.pose_(2, 3) = key_frame_msgs_ptr->pose.pose.position.z;
        /*拷贝旋转量*/
        Eigen::Quaternionf q;
        q.x() = key_frame_msgs_ptr->pose.pose.orientation.x;
        q.y() = key_frame_msgs_ptr->pose.pose.orientation.y;
        q.z() = key_frame_msgs_ptr->pose.pose.orientation.z;
        q.w() = key_frame_msgs_ptr->pose.pose.orientation.w;
        /*四元数转旋转矩阵*/
        key_frame.pose_.block<3, 3>(0, 0) = q.matrix();
        /*新加入队列*/
        new_key_frame_buff_.push_back(key_frame);
    }

} // namespace multisensor_localization