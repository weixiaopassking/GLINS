#ifndef _POINTCLOUD_HANDLE_HPP
#define _POINTCLOUD_HANDLE_HPP

#include <pcl/io/pcd_io.h>   //读写pcd
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //单点

namespace quadrotor_test
{
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

class PointCloudHandle
{
  public:
    PointCloudHandle(){};
    PointCloudHandle(std::string pcd_path);
    // PointCloudHandle(PointCloudType pointcloud_source);
    // PointCloudHandle(PointCloudType pointcloud_source, PointCloudType pointcloud_target);

  private:
    void Pcd2Bev();
    // void KNN();
    // void OctTree();

    PointCloudType::Ptr _point_cloud_ptr;
};
} // namespace quadrotor_test

#endif //_POINTCLOUD_HANDLE_HPP