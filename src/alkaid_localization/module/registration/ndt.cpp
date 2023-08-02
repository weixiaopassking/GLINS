#include "ndt.hpp"

namespace module_ns
{

NDT::NDT()
{
    // todo add yaml config
    float res = 1.0;        // node["res"].as<float>();
    float step_size = 0.1;  // node["step_size"].as<float>();
    float trans_eps = 0.01; // node["trans_eps"].as<float>();
    int max_iter = 30;      // node["max_iter"].as<int>();

    _ndt_ptr->setResolution(res);
    _ndt_ptr->setStepSize(step_size);
    _ndt_ptr->setTransformationEpsilon(trans_eps);
    _ndt_ptr->setMaximumIterations(max_iter);
}

void NDT::SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr)
{

    _ndt_ptr->setInputSource(source_cloud_ptr);
}

void NDT::SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr)
{

    _ndt_ptr->setInputTarget(target_cloud_ptr);
}

void NDT::SetGtTransform(const data_ns::Mat4f &gt_transform)
{
}

bool NDT::GetResTransform(data_ns::Mat4f &init_transform)
{
    data_ns::CloudData::CLOUD_PTR result_cloud_ptr(new data_ns::CloudData::CLOUD);
    _ndt_ptr->align(*result_cloud_ptr, init_transform);
    init_transform = _ndt_ptr->getFinalTransformation();
    return true;
}

NDT::~NDT()
{
}
} // namespace module_ns