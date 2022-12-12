/*
 * @Description: preprocess sensor data
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 */

#ifndef PREPROCESS_FLOW_HPP_
#define PREPROCESS_FLOW_HPP_

// ros lib
#include <ros/ros.h>
// sub
#include "../subscriber/ImuSubscriber.hpp"
#include "../subscriber/GnssFixSubscriber.hpp"
#include "../subscriber/CloudSubscriber.hpp"
// pub
#include "../publisher/CloudPublisher.hpp"
#include "../publisher/OdomPublisher.hpp"
#include "../publisher/EnuPublisher.hpp"
#include "../publisher/ImuPublisher.hpp"

// thirdpart lib
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace glins
{
    class PreprocessFlow
    {
    public:
        PreprocessFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool TimeSynchronization();
        bool SpaceCalibration();
        bool InitEnuOrigin();
        
        bool CheckDataQueue();
        bool ExtractData();

        void ImuPreintegration();
        void PublishData();

    private:
        /*raw sensor data subscrier*/
        std::shared_ptr<ImuSubscriber> imu_sub_ptr_;
        std::shared_ptr<GnssFixSubscriber> gnss_fix_sub_ptr_;
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        /*synced sensor data publisher*/
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<OdomPublisher> gnss_fix_pub_ptr_;
        std::shared_ptr<EnuPublisher> enu_origin_pub_ptr_;
        std::shared_ptr<ImuPublisher> imu_pub_ptr_;
        std::shared_ptr<OdomPublisher> imu_odom_pub_ptr_;
        /*yaml node*/
        YAML::Node config_node_;
        /*flag for setting*/
        bool gnss_enable_ = false;

        /*sensor data current and queue*/
    }; // class PreprocessFlow
} // namespace glins

#endif