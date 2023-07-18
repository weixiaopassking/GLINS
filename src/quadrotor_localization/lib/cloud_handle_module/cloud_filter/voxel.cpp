/**
 * @brief 体素滤波器
 * @param cloud_ptr 点云指针
 * @param voxel_size 滤波器尺寸
 * @return void
 * @note 静态成员函数
 */
void PointCloudHandle::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const float voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZI> filter_voxel;
    filter_voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter_voxel.setInputCloud(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    filter_voxel.filter(*cloud_filtered_ptr);
    cloud_ptr->swap(*cloud_filtered_ptr);
}