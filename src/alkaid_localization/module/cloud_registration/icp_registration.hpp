#ifndef _ICP_REGISTRATON_HPP
#define _ICP_REGISTRATON_HPP

#include "cloud_registration_interface.hpp"
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include <numeric>
#include <pcl/kdtree/kdtree_flann.h>
#include <execution>
#include <vector>

namespace module_ns
{

class ICPRegistration : public CloudRegistrationInterface
{

  public:
    enum ICPMethods
    {
        point2point,
        point2line,
        point2lane
    };

    struct Options
    {
        const int max_iteration = 30;
        const int min_nn_numbers = 10;             // 最小最近邻点数
        const bool use_initial_translation = true; // 是否使用平移初始值

        const double max_point2point_distance = 1.0;
        const double max_point2line_distance = 0.5;
        const double max_point2plane_distance = 0.05;

        const double epsilon = 1e-3;

        ICPMethods icp_method = ICPMethods::point2point;
    };

    ICPRegistration();
    void SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr) override;
    void SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr) override;
    void SetGtTransform(const data_ns::Mat4f &gt_transform) override;
    data_ns::Mat4f GetResTransform(const data_ns::Mat4f &predict_transform) override;

    ~ICPRegistration() = default;

  private:
    data_ns::Mat4f Point2Point(const data_ns::Mat4f &predict_transform);
    data_ns::Mat4f Point2Plane(const data_ns::Mat4f &predict_transform);
    data_ns::Mat4f Point2Line(const data_ns::Mat4f &predict_transform);

    data_ns::CloudData::CLOUD_PTR _source_cloud_ptr;
    data_ns::CloudData::CLOUD_PTR _target_cloud_ptr;

    data_ns::Vec3f _source_center_vec;
    data_ns::Vec3f _target_center_vec;

    Options _option;

    data_ns::SE3f _gt_transform;
    bool _has_gt_transform;

    typedef Eigen::Matrix<float, 6, 6> H_type; // gauss H=JJ^T
    typedef Eigen::Matrix<float, 6, 1> g_type;
    typedef Eigen::Matrix<float, 6, 1> dx_type;
};
} // namespace module_ns
#endif //_ICP_REGISTRATON_HPP