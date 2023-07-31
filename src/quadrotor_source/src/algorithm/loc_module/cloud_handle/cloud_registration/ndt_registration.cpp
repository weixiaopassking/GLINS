#include "./ndt_registration.hpp"

namespace algorithm_ns
{

void NDTRegistration::SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr)
{
    _ndt_pcl.setInputSource(source_cloud_ptr);
}


void NDTRegistration::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr)
{
    _ndt_pcl.setInputTarget(target_cloud_ptr);
}

void NDTRegistration::SetGtTransform(const Sophus::SE3d &gt_transform)
{

}


bool NDTRegistration::GetResTransform(Sophus::SE3d &init_transform)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>());
    _ndt_pcl.align(*output, init_transform.matrix().cast<float>());

    Eigen::Matrix<double, 4, 4> m = _ndt_pcl.getFinalTransformation().cast<double>().eval();

    Eigen::Quaterniond q(m.block<3, 3>(0, 0).cast<double>());
    q.normalize();
    Sophus::SE3d temp(q, m.block<3, 1>(0, 3).cast<double>());

    init_transform = temp;
}




NDTRegistration::~NDTRegistration()
{
}
} // namespace algorithm_ns