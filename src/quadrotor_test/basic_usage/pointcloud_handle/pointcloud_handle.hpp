#ifndef _POINT_CLOUD_HPP
#define _POINT_CLOUD_HPP

/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    pointcloud_handle.cpp
*  @brief  点云处理
*  @author  robotics gang
*  @date    2023/7/11
*  @version v0.1
*  @ref  github.com/gaoxiang12/slam_in_autonomous_driving
****************************************************************************
*/

#include <Eigen/Core> //矩阵定义
#include <numeric>
#include <opencv2/core/core.hpp> //图像核心处理
#include <opencv2/highgui/highgui.hpp>
#include <ostream>
#include <pcl/io/pcd_io.h>                    //读写pcd
#include <pcl/point_cloud.h>                  //点云
#include <pcl/point_types.h>                  //单点
#include <pcl/visualization/pcl_visualizer.h> //可视化调用
#include <string>

class PointCloudHandle
{
  public:
    PointCloudHandle() = delete;
    PointCloudHandle(const std::string pcd_path);
    PointCloudHandle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_ptr);
    PointCloudHandle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_ptr);

    ~PointCloudHandle();

    void GenerateBevImage(const double image_resolution = 0.10, const double z_upper = 2.50,
                          const double z_lower = 0.20);

    static bool PlaneFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 4, 1>&plane_coeffs,const double eps=1e-2);
    static bool LineFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 3, 1> &start_point,
                            Eigen::Matrix<double, 3, 1> &direction, const double eps = 0.2);
    void Knn();
    void Display();

    friend std::ostream &operator<<(std::ostream &o, const PointCloudHandle &s);

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_source_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_target_ptr;
};

#endif // _POINT_CLOUD_HPP