/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    cloud_io.hpp
*  @brief  点云的读写
*  @author  robotics gang
*  @date    2023/7/18
*  @version v0.1
*  @ref https://github.com/gaoxiang12
****************************************************************************
*/

#include <vector>
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点
#include <numeric>//数值运算

class CloudFitting
{
public:
  static bool PlaneFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 4, 1> &plane_coeffs,
                           const double eps = 1e-2); 
  static bool LineFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 3, 1> &start_point,
                          Eigen::Matrix<double, 3, 1> &direction, const double eps = 0.2); 
};