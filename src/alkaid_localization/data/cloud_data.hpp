#ifndef _CLOUD_DATA_HPP
#define _CLOUD_DATA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace data_ns
{
class CloudData
{
  public:
  //defination for cloud type used in the package
    using POINT = pcl::PointXYZI;
    using CLOUD = pcl::PointCloud<pcl::PointXYZI>;
    using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZI>::Ptr;

  public:
    CloudData();

  public:
    double _time_stamp = 0.0;//sec use unix time
    CLOUD_PTR _cloud_ptr;
    
}; // class CloudData
} // namespace data_ns

#endif //_CLOUD_DATA_HPP