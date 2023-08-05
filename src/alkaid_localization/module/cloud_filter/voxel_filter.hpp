#include "cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace module_ns
{
class VoxelFilter : public CloudFilterInterface
{
  public:
    VoxelFilter()=delete;
     VoxelFilter(float leaf_szie);
    bool Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr) override;
    ~VoxelFilter() = default;

  private:
    pcl::VoxelGrid<data_ns::CloudData::POINT> _voxel_filter;
};
} // namespace module_ns
