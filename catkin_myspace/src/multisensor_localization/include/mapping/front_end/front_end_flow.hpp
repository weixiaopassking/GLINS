/*
 * @Description: 前端任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-17
 */

#ifndef FRONT_END_FLOW_HPP_
#define FRONT_END_FLOW_HPP_

// ros
#include <ros/ros.h>
//订阅器
#include "../../subscriber/cloud_subscriber.hpp"
//发布器
#include "../../publisher/odometry_publisher.hpp"
//前端算法
#include "../../mapping/front_end/front_end.hpp"
// glog
#include <glog/logging.h>

namespace multisensor_localization
{
    class FrontEndFlow
    {
    public:
        FrontEndFlow(ros::NodeHandle &nh);

        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateLaserOdometry();
        bool PublishData();

    private:
        /*订阅点云*/
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        /*发布激光里程计*/
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        /*前端算法*/
        std::shared_ptr<FrontEnd> front_end_ptr_;
        /*点云缓冲*/
        std::deque<CloudData> cloud_data_buff_;
        /*当前点云*/
        CloudData current_cloud_data_;
        /*激光里计T矩阵*/
        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
    };
}

#endif