#include "../cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

class VoxelFilter : public CloudFilterInterface
{
  public:
    VoxelFilter(const YAML::Node &node);
    VoxelFilter(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z);
    void Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud_ptr) override;

  private:

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};
