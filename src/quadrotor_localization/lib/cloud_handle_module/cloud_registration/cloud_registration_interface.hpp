#ifndef _CLOUD_REGISTRATION_HPP
#define _CLOUD_REGISTRATION_HPP

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点

class CloudRegistrationInterface
{
  public:
    CloudRegistrationInterface(){};
    virtual void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud_ptr) = 0;
    virtual void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud_ptr) = 0;
    virtual void SetGtTransform(const Sophus::SE3d &res_transform) = 0;
    virtual Sophus::SE3d  GetResTransform() = 0;
    virtual ~CloudRegistrationInterface(){};
};

#endif //_CLOUD_REGISTRATION
