#include "ndt_registration.hpp"

namespace module_ns
{

NDTRegistration::NDTRegistration()
    : _ndt_registration_ptr(
          new pcl::NormalDistributionsTransform<data_ns::CloudData::POINT, data_ns::CloudData::POINT>())
{
   
    // todo add yaml config
    float res = 1.0;        // node["res"].as<float>();
    float step_size = 0.1;  // node["step_size"].as<float>();
    float trans_eps = 0.01; // node["trans_eps"].as<float>();
    int max_iter = 30;      // node["max_iter"].as<int>();

    _ndt_registration_ptr->setResolution(res);
    _ndt_registration_ptr->setStepSize(step_size);
    _ndt_registration_ptr->setTransformationEpsilon(trans_eps);
    _ndt_registration_ptr->setMaximumIterations(max_iter);
}

void NDTRegistration::SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr)
{

    _ndt_registration_ptr->setInputSource(source_cloud_ptr);
}

void NDTRegistration::SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr)
{

    _ndt_registration_ptr->setInputTarget(target_cloud_ptr);
}

void NDTRegistration::SetGtTransform(const data_ns::Mat4f &gt_transform)
{
}

bool NDTRegistration::GetResTransform(data_ns::Mat4f &init_transform)
{
    data_ns::CloudData::CLOUD_PTR result_cloud_ptr(new data_ns::CloudData::CLOUD);
    _ndt_registration_ptr->align(*result_cloud_ptr, init_transform);
    init_transform = _ndt_registration_ptr->getFinalTransformation();
    return true;
}

NDTRegistration::~NDTRegistration()
{
}
} // namespace module_ns