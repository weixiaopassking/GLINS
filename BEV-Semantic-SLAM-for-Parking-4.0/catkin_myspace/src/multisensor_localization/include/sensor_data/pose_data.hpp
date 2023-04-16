/*
 * @Description: 位姿势数据存放
 * @Function: 位姿+时间戳
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo 
 */

#ifndef POSE_DATA_HPP_
#define POSE_DATA_HPP_

#include <Eigen/Dense>

namespace multisensor_localization
{

    class PoseData
    {
    public:
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
        double time_stamp_ = 0.0;

        public:
        Eigen::Quaternionf GetQuaternion();
    };

} // namesapce multisensor_localization

#endif
