/*
 * @Description: 东北天坐标系原点发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef  ORIGIN_PUBLISHER_HPP_
#define ORIGIN_PUBLISHER_HPP_

//ros
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
//自定义gnss数据
#include "../sensor_data/gnss_data.hpp"


namespace multisensor_localization
{
   class OriginPublisher
    {
    public:
        OriginPublisher(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string frame_id);
        OriginPublisher() = default;

        void Publish(GnssData & gnss_input );
        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        sensor_msgs::NavSatFix origin_;
        std::string frame_id_;
    };

}//namespace multisensor_localization

#endif