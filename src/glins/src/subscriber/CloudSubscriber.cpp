/*
 * @Description: receive gnss fixed from gnss device
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-04
 */

// relevent
#include "../../include/subscriber/CloudSubscriber.hpp"
// thirdpart lib
#include <pcl_conversions/pcl_conversions.h>

namespace glins
{
    /**
     * @brief  config ros topic
     * @note refer to
     * @todo
     **/
    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size)
    {
        subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(topic_name, queue_size, &CloudSubscriber::MsgCallbcak, this, ros::TransportHints().tcpNoDelay());
    }

    /**
     * @brief  callback function
     * @note push the data into buff
     * @todo
     **/
    void CloudSubscriber::MsgCallbcak(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        cloud_sub_mutex_.lock();

        CloudDataType cloud_data;
        cloud_data.time_stamp = msg->header.stamp.toSec();
        pcl::fromROSMsg(*msg, *(cloud_data.cloud_ptr));

        data_buffer_.push_back(cloud_data);

        cloud_sub_mutex_.unlock();
    }

    /**
     * @brief  read data from buff
     * @note read and clear the buff
     * @todo
     **/
    void CloudSubscriber::ParseData(std::deque<CloudDataType> &data_queue)
    {
        if (data_buffer_.size() > 0)
        {
            data_queue.insert(data_queue.end(), data_buffer_.begin(), data_buffer_.end());
            data_buffer_.clear();
        }
    }
} // namespace glins