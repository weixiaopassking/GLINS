#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class CloudFilterInterface
{
  public:
    CloudFilterInterface() = delete;
    virtual bool Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) = 0;
};