/*
 * @Description: 回环数据类型
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 * @Todo
 */

#ifndef LOOP_CLOSING_HPP_
#define LOOP_CLOSING_HPP_

#include <Eigen/Dense>

namespace multisensor_localization
{

    class LoopPose
    {
    public:
    double time_stamp_;
    unsigned int index0_=0;
    unsigned int index1=0;
    Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
    public:
    Eigen::Quaternionf GetQuaternion();
    }; // class LoopPose

}; // namespace multisensor_localization

#endif