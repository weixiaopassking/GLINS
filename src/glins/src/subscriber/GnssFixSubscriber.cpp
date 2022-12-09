/*
 * @Description:gnss fix subscriber
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 */

// relevent
#include "../../include/subscriber/GnssFixSubscriber.hpp"

namespace glins
{
    /**
     * @brief  config ros topic
     * @note refer to https://docs.ros.org/en/fuerte/api/sensor_msgs/html/msg/NavSatFix.html
     * @todo
     **/
    GnssFixSubscriber::GnssFixSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe<sensor_msgs::NavSatFix>(topic_name, queue_size, &GnssFixSubscriber::MsgCallbcak, this, ros::TransportHints().tcpNoDelay());
    }

    /**
     * @brief  callback function
     * @note push the data into buff
     * @todo
     **/
    void GnssFixSubscriber::MsgCallbcak(const sensor_msgs::NavSatFixConstPtr &msg)
    {
        gnss_fix_sub_mutex_.lock();

        GnssFixDataType gnss_fix_data;

        gnss_fix_data.time_stamp = msg->header.stamp.toSec();

        gnss_fix_data.longtitude = msg->longitude;
        gnss_fix_data.latitude = msg->latitude;
        gnss_fix_data.altitude = msg->altitude;

        gnss_fix_data.status = msg->status.status;
        gnss_fix_data.service = msg->status.service;

        data_buffer_.push_back(gnss_fix_data);

         gnss_fix_sub_mutex_.unlock();
    }

    /**
     * @brief  read data from buff
     * @note read and clear the buff
     * @todo
     **/
    void GnssFixSubscriber::ParseData(std::deque<GnssFixDataType> &data_queue)
    {
        if (data_buffer_.size() > 0)
        {
            data_queue.insert(data_queue.end(), data_buffer_.begin(), data_buffer_.end());
            data_buffer_.clear();
        }
    }

} // namespace glins