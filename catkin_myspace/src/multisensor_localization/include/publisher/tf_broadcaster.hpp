/*
 * @Description: tf发布
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-14
 */

#ifndef TF_BROADCASTER_HPP_
#define TF_BROADCASTER_HPP_

// c++
#include <string>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
// tf树
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace multisensor_localization
{

    class TfBroadcaster
    {
    public:
        TfBroadcaster(std::string frame_id, std::string child_id);
        TfBroadcaster() = default;
        void SendTransform(Eigen::Matrix4f pose, double time_stamp);
    private:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;

    }; // TfBroadcaster

} // namespace multisensor_localization

#endif