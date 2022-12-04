/*
 * @Description: receive gnss fixed from gnss device
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-04
 */

// relevent
#include "../../include/subscriber/CloudSubscriber.hpp"

namespace glins
{

    CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size)
    {
        subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(topic_name, queue_size, &CloudSubscriber::MsgCallbcak, this, ros::TransportHints().tcpNoDelay());
    }
    void CloudSubscriber::ParseData(std::deque<CloudData> &data_deque)
    {
        
    }

    void CloudSubscriber::MsgCallbcak(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {

    }

} // namespace glins