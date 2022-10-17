/*
 * @Description:点云匹配算法
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#ifndef REGISTRATION_INTERFACE_HPP_
#define REGISTRATION_INTERFACE_HPP_

// 自定义点云数据类型
#include "../../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    class RegistrationInterface
    {
    public:
        virtual ~RegistrationInterface() = default;
        virtual bool SetInputTarget(const CloudData::CLOUD_PTR &input_cloud) = 0;
        virtual bool ScanMatch(const CloudData::CLOUD_PTR &input_cloud_ptr,
                               const Eigen::Matrix4f &predict_pose,
                               CloudData::CLOUD_PTR &result_cloud_ptr,
                               Eigen::Matrix4f &result_pose) = 0;
    };

} // namespace multisensor_localization

#endif