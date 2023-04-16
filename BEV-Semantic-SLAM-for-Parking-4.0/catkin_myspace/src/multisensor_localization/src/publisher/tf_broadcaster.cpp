/*
 * @Description: 东北天坐标系原点发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#include "../../include/publisher/tf_broadcaster.hpp"

namespace multisensor_localization
{
    TfBroadcaster::TfBroadcaster(std::string frame_id, std::string child_id)
    {
        transform_.frame_id_ = frame_id;
        transform_.child_frame_id_ = child_id;
    }

    void TfBroadcaster::SendTransform(Eigen::Matrix4f pose, double time_stamp)
    {
        Eigen::Quaternionf  q(pose.block<3, 3>(0, 0));
        ros::Time ros_time((float)time_stamp);
        transform_.stamp_ = ros_time;
        transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
        broadcaster_.sendTransform(transform_);
    }

} // multisensor_localizationsss