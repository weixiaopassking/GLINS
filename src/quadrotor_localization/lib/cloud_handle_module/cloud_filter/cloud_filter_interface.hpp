#ifndef _CLOUD_FILTER_INTERFACE_HPP
#define _CLOUD_FILTER_INTERFACE_HPP


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudFilterInterface
{
  public:
    CloudFilterInterface() = default;
    virtual void  Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud_ptr) = 0;
    ~CloudFilterInterface() = default;
};

#endif //_CLOUD_FILTER_INTERFACE_HPP
