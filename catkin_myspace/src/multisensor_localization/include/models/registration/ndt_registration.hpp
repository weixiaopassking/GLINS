/*
 * @Description:NDT匹配算法
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#ifndef NDT_REGISTRATION_HPP_
#define NDT_REGISTRATION_HPP_

// relevent
#include "./registration_interface.hpp"
// pcl
#include <pcl/registration/ndt.h>
// yaml
#include <yaml-cpp/yaml.h>

namespace multisensor_localization
{
  class NdtRegistration : public RegistrationInterface
  {
  public:
    NdtRegistration(const YAML::Node &node);
    NdtRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR &input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR &input_cloud_ptr,
                   const Eigen::Matrix4f &predict_pose,
                   CloudData::CLOUD_PTR &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;

  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
  };

} // namespace multisensor_localization

#endif