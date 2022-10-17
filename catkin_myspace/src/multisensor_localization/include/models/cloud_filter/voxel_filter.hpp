/*
 * @Description:体素滤波器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#ifndef VOXEL_FILTER_HPP_
#define VOXEL_FILTER_HPP_

//relevent
#include "./cloud_filter_interface.hpp"
//pcl
#include <pcl/filters/voxel_grid.h>

namespace multisensor_localization
{

    class VoxelFilter : public CloudFilterInterface
    {
    public:
        VoxelFilter(const YAML::Node &node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
        bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr) override;

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };

} //namespace multisensor_localization

#endif
