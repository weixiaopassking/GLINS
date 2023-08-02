#include "filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace module_ns
{
class Voxel : public FilterInterface
{
  public:
    Voxel();
    bool Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr) override;
    ~Voxel() = default;

  private:
    pcl::VoxelGrid<data_ns::CloudData::POINT> _voxel;
};
} // namespace module_ns
