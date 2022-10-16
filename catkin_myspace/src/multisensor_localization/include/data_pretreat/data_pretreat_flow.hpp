/*
 * @Description: 传感器数据处理任务管理
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef DATA_PRETREAT_FLOW_HPP_
#define DATA_PRETREAT_FLOW_HPP_

// ros
#include <ros/ros.h>
// yaml
#include <yaml-cpp/yaml.h>
// subscriber
#include "../subscriber/cloud_subscriber.hpp"
#include "../subscriber/gnss_subscriber.hpp"
#include "../subscriber/imu_subscriber.hpp"
#include "../subscriber/velocity_subscriber.hpp"
// publisher
#include "../publisher/cloud_publisher.hpp"
#include "../publisher/odometry_publisher.hpp"
#include "../publisher/origin_publisher.hpp"
//畸变矫正
#include "../models/scan_adjust/distortion_adjust.hpp"

//畸变矫正 todo but 可能有点问题

namespace multisensor_localization
{
    class DataPretreatFlow
    {
    public:
        DataPretreatFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool InitCalibration();
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool TransformData();
        bool PublishData();

    private:
        /*传感器数据订阅*/
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<GnssSubscriber> gnss_sub_ptr_;
        /*数据发布*/
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        std::shared_ptr<OriginPublisher> origin_pub_ptr_;
        //畸变矫正
        std::shared_ptr<DistortionAdjust> distortion_adjust_;
        /*数据队列*/
        std::deque<CloudData> cloud_data_buff_;
        std::deque<ImuData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GnssData> gnss_data_buff_;

        Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

        YAML::Node config_node_;
        /*当前数据*/
        CloudData current_cloud_data_;
        ImuData current_imu_data_;
        VelocityData current_velocity_data_;
        GnssData current_gnss_data_;
    };

} // namespace multisensor_localization

#endif