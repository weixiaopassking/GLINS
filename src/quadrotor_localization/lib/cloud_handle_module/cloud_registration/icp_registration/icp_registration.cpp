#include "./icp_registration.hpp"

ICPRegistration::ICPRegistration()
{
    this->_source_cloud_ptr.reset();
    this->_target_cloud_ptr.reset();
    has_gt_transform = false;
}

void ICPRegistration::SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr)
{
    this->_source_cloud_ptr = source_cloud_ptr;
    // this->_source_center_vec=std::accumulate();
}

void ICPRegistration::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr)
{

    this->_target_cloud_ptr = target_cloud_ptr;
}

void ICPRegistration::SetGtTransform(const Sophus::SE3d &gt_transform)
{
    this->_gt_transform = gt_transform;
    has_gt_transform = false;
}

Sophus::SE3d ICPRegistration::GetResTransform()
{
    // return
}

ICPRegistration::~ICPRegistration()
{
}