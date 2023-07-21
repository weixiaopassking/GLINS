/*
 * @Description:体素滤波器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#include "./voxel_filter.hpp"

VoxelFilter::VoxelFilter(const YAML::Node &node)
{
//todo 从yaml读
// voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

/**
 * @brief 点云滤波参数设置
 * @todo
 * @note
 **/
VoxelFilter::VoxelFilter(const float leaf_size_x, const float leaf_size_y, const float leaf_size_z)
{
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
}

/**
 * @brief 点云滤波
 * @todo
 * @note
 **/
void VoxelFilter::Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ptr,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud_ptr)
{
    voxel_filter_.setInputCloud(cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);
}