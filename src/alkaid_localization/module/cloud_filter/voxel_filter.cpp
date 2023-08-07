/*
 * @Description: cloud filter interface
 * @Function: as interface to use different menthods
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 * @Note: supported ndt_pcl、p-p-icp
 */

#include "./voxel_filter.hpp"

namespace module_ns
{

/**
 * @brief   voxel init
 * @param leaf_size 0.6 for nclt
 * @note
 **/
VoxelFilter::VoxelFilter(const float leaf_size)
{
    float leaf_size_x = leaf_size; 
    float leaf_size_y = leaf_size;
    float leaf_size_z = leaf_size;

    _voxel_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

/**
 * @brief   voxel filter
 * @param
 * @note
 **/
bool VoxelFilter::Filter(const data_ns::CloudData::CLOUD_PTR &cloud_ptr,
                         data_ns::CloudData::CLOUD_PTR &filtered_cloud_ptr)
{
    _voxel_filter.setInputCloud(cloud_ptr);
    _voxel_filter.filter(*filtered_cloud_ptr);
    return true;
}
} // namespace module_ns
