/*
 * @Description: receive gnss fixed from gnss device
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-12-04
 */

// relevent
#include "../sensor_data/CloudDataType.hpp"
//ros ib
#include <ros/ros.h>
# include <sensor_msgs/PointCloud2.h>
//c++ lib
#include <deque>

namespace glins
{
    class CloudSubscriber
    {
    public:
        CloudSubscriber() = default;
        CloudSubscriber(ros::NodeHandle &nh, const std::string topic_name, const size_t queue_size);
        void ParseData(std::deque<CloudDataType> &data_queue);

    private:
        void MsgCallbcak(const sensor_msgs::PointCloud2::ConstPtr &msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<CloudDataType> data_buffer_;
    }; // class glins
} // namespace glins