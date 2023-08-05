#ifndef  _CLOUD_REGISTRATION_INTERFACE_HPP
#define _CLOUD_REGISTRATION_INTERFACE_HPP

#include "../../data/cloud_data.hpp"
#include "../../data/geometry_data.hpp"

#include <pcl/point_cloud.h> //pointcloud
#include <pcl/point_types.h> //point

namespace module_ns
{

class CloudRegistrationInterface
{
  public:
    CloudRegistrationInterface() = default;
    virtual void SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr) = 0;
    virtual void SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr) = 0;
    virtual void SetGtTransform(const data_ns::Mat4f &res_transform) = 0;
    virtual data_ns::Mat4f GetResTransform(const data_ns::Mat4f &predict_transform) = 0;
    virtual ~CloudRegistrationInterface() = default; // must be  set virtual  to release inherit class's memory
};
} // namespace module_ns
#endif // _REGISTRATION_HPP
