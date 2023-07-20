#include "../cloud_filter_interface.hpp"



    class VoxelFilter : public CloudFilterInterface
    {
        Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr,
               pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud_ptr);
    }
          