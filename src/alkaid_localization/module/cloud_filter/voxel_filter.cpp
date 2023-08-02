#include "./voxel_filter.hpp"

namespace module_ns
{
VoxelFilter::VoxelFilter()
{
    float leaf_size_x = 1.3;
    float leaf_size_y = 1.3;
    float leaf_size_z = 1.3;

    _voxel_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}
bool VoxelFilter::Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                         data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr)
{
    _voxel_filter.setInputCloud(cloud_ptr);
    _voxel_filter.filter(*filtered_cloud_ptr);
    return true;
}
} // namespace module_ns
