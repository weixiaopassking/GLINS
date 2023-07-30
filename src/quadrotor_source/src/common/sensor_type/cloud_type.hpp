#ifndef _CLOUD_TYPE_HPP
#define _CLOUD_TYPE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



namespace common_ns
{

class CloudType
{

  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<pcl::PointXYZI>;
    using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZI>::Ptr;

    CloudType();

  public:
    double _timestamp = 0.0;
    CLOUD_PTR _cloud_ptr;

}; // class CloudType

} // namespace common_ns

#endif