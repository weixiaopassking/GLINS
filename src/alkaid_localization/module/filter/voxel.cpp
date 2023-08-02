#include "./voxel.hpp"

namespace module_ns
{
Voxel::Voxel()
{
    float leaf_size_x = 1.3;
    float leaf_size_y = 1.3;
    float leaf_size_z = 1.3;

    _voxel.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}
bool Voxel::Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr, data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr)
{
    _voxel.setInputCloud(cloud_ptr);
    _voxel.filter(*filtered_cloud_ptr);
    return true;
}
} // namespace module_ns
