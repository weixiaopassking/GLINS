/*
 * @Description: cloud filter interface
 * @Function: as interface to use different menthods
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 * @Note: supported ndt_pcl、p-p-icp
 */

#include "cloud_filter_interface.hpp"
#include <pcl/filters/voxel_grid.h>

namespace module_ns
{
class VoxelFilter : public CloudFilterInterface
{
  public:
    VoxelFilter()=delete;
     VoxelFilter(const float leaf_szie);
    bool Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr) override;
    ~VoxelFilter() = default;

  private:
    pcl::VoxelGrid<data_ns::CloudData::POINT> _voxel_filter;
}; // class VoxelFilter
} // namespace module_ns
