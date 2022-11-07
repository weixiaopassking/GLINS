/*
 * @Description: 关键帧数据封装
 * @Function: 模块间传递数据 位姿+时间戳+序列
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo 
 */

#ifndef KEY_FRAME_HPP_
#define KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace multisensor_localization
{

    class KeyFrame
    {
    public:
        Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
        double time_stamp_ = 0.0;
        unsigned int index_= 0;

    public:
        Eigen::Quaternionf GetQuaternion();
    };

} // namesapce multisensor_localization

#endif