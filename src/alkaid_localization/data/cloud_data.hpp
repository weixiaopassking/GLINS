#ifndef _CLOUD_DATA_HPP
#define _CLOUD_DATA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace data_ns
{
class CloudData
{
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<pcl::PointXYZ>;
    using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  public:
    CloudData();

  public:
    double _time_stamp = 0.0;
    CLOUD_PTR _cloud_ptr;
    
}; // class CloudData
} // namespace data_ns

#endif //_CLOUD_DATA_HPP