/*
 * @Description:ndt registration
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#include "ndt_pcl_registration.hpp"

namespace module_ns
{

/**
 * @brief   ndt init
 * @param 
 * @note
 **/
NDTPclRegistration::NDTPclRegistration()
    : _ndt_registration_ptr(
          new pcl::NormalDistributionsTransform<data_ns::CloudData::POINT, data_ns::CloudData::POINT>())
{

    _ndt_registration_ptr->setResolution(_option.resolution);
    _ndt_registration_ptr->setStepSize(_option.step_size);
    _ndt_registration_ptr->setTransformationEpsilon(_option.epsilon);
    _ndt_registration_ptr->setMaximumIterations(_option.max_iteration);
}

/**
 * @brief   ndt set source
 * @param
 * @note
 **/
void NDTPclRegistration::SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr)
{
    _ndt_registration_ptr->setInputSource(source_cloud_ptr);
}

/**
 * @brief   ndt set target
 * @param
 * @note
 **/
void NDTPclRegistration::SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr)
{
    _ndt_registration_ptr->setInputTarget(target_cloud_ptr);
}

/**
 * @brief   ndt set gt transform
 * @param
 * @note
 **/
void NDTPclRegistration::SetGtTransform(const data_ns::Mat4f &gt_transform)
{
}

/**
 * @brief   ndt get result of tansform
 * @param
 * @note
 **/
data_ns::Mat4f NDTPclRegistration::GetResTransform(const data_ns::Mat4f &predict_transform)
{
    data_ns::CloudData::CLOUD_PTR result_cloud_ptr(new data_ns::CloudData::CLOUD);
    _ndt_registration_ptr->align(*result_cloud_ptr, predict_transform);
    // _ndt_registration_ptr->align(*result_cloud_ptr);
    data_ns::Mat4f res_transform;
    res_transform.matrix() = _ndt_registration_ptr->getFinalTransformation();
    return res_transform;
}


} // namespace module_ns