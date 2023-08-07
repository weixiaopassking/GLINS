/*
 * @Description:ndt registration
 * @Function: 
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _NDT_PCL_REGISTRATON_HPP
#define _NDT_PCL_REGISTRATON_HPP

#include "cloud_registration_interface.hpp"
#include <pcl/registration/ndt.h>

namespace module_ns
{

class NDTPclRegistration : public CloudRegistrationInterface
{
    struct Options
    {
        float resolution = 0.5;
        float step_size = 0.1;
        float epsilon = 1e-3;
        int max_iteration = 30;
    };

  public:
    NDTPclRegistration();
    void SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr) override;
    void SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr) override;
    void SetGtTransform(const data_ns::Mat4f &gt_transform) override;
    data_ns::Mat4f GetResTransform(const data_ns::Mat4f &predict_transform) override;
    ~NDTPclRegistration() = default;

  private:
    Options _option;
    pcl::NormalDistributionsTransform<data_ns::CloudData::POINT, data_ns::CloudData::POINT>::Ptr _ndt_registration_ptr;
}; // class NDTRegistration
} // namespace module_ns

#endif //_NDT_REGISTRATON_HPP
