/*
 * @Description: 重定位任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-08
 */

#ifndef MATCHING_FLOW_HPP_
#define MATCHING_FLOW_HPP_

// ros
#include <ros/ros.h>
// sub
#include "../../include/subscriber/cloud_subscriber.hpp"
#include "../../include/subscriber/odometry_subscriber.hpp"
// pub
#include "../../include/publisher/odometry_publisher.hpp"
#include "../../include/publisher/cloud_publisher.hpp"
#include "../../include/publisher/tf_broadcaster.hpp"
//算法层
#include "../../include/matching/matching.hpp"

namespace multisensor_localization
{
    class MatchingFlow
    {
    public:
        MatchingFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateMatching();
        bool PublishData();

        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;

        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
        std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<TfBroadcaster> laser_tf_pub_ptr_;

        std::shared_ptr<Matching> matching_ptr_;

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_data_buff_;

        CloudData current_cloud_data_;
        PoseData current_gnss_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    }; // class MatchingFlow

} // namespace multisensor_localization

#endif