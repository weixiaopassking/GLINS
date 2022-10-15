/*
 * @Description: 里程计数据订阅
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-15
 */

#ifndef  ODOMETRY_SUBSCRIBER_HPP_
#define ODOMETRY_SUBSCRIBER_HPP_

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
//c++
#include <deque>
//自定义位姿数据
#include "../sensor_data/pose_data.hpp"


namespace multisensor_localization
{

    class OdometrySubscriber
    {
        public:
        OdometrySubscriber(ros::NodeHandle&nh,std::string topic_name,size_t buff_size);
        OdometrySubscriber()=default;
       void ParseData(std::deque<PoseData>&pose_data_buff);

        private:
        void MsgCallback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

        private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<PoseData> new_pose_data_buff_;
    };

} // namespace multisensor_localization

#endif