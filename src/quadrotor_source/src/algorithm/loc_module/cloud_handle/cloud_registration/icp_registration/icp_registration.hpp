/**
*****************************************************************************
*  Copyright (C), 2023-2026,wengang.niu
*  @file    icp_registration.hpp
*  @brief  点云配准icp
*  @author  wengang.niu
*  @date    2023/7/26
*  @version v0.2
*  @ref github:gaoxiang12/slam_in_autonomous_driving
*  @note
算法原理:
求出雅可比矩阵
高斯牛顿法不断迭代求解Rt
note: 适配point-point、point-plane 、point-line
调用步骤:
a. SetSourceCloud 设置源点云
b. SetTargetCloud 设置目标点云
c. SetGtTransform 设置真值
d. GetResTransform  获取点云
****************************************************************************
*/

#include "../cloud_registration_interface.hpp"
#include <execution>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

namespace algorithm_ns
{



class ICPRegistration : public CloudRegistrationInterface
{

  public:
    enum icp_methods
    {
        point2point,
        point2line,
        point2lane
    };
 
     struct Options
    {
        const int max_iteration = 100;
        const int min_nn_numbers = 10;//最小最近邻点数
        const bool use_initial_translation=false; //是否使用平移初始值

        const double max_point2point_distance = 1.0;
        const double max_point2line_distance = 0.5;
        const double max_point2plane_distance = 0.05;

        const double epsilon = 1e-4;//迭代阈值
        icp_methods icp_method = icp_methods::point2point;
    };
    ICPRegistration();

    void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr) override;
    void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) override;
    void SetGtTransform(const Sophus::SE3d &gt_transform) override;
    bool GetResTransform(Sophus::SE3d &init_transform) override;

    ~ICPRegistration();

  private:
    bool Point2Point(Sophus::SE3d &init_transform); // 点到点ICP
    bool Point2Plane(Sophus::SE3d &init_transform); // 点到面ICP
    bool Point2Line(Sophus::SE3d &init_transform);  // 点到线ICP

    pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;
    Sophus::SE3d _gt_transform;
    bool has_gt_transform;

    Eigen::Vector3d _source_center_vec;
    Eigen::Vector3d _target_center_vec;


    Options _options;//选项卡

    typedef Eigen::Matrix<double, 6, 6> H_type;//高斯牛顿H=JJ^T
    typedef Eigen::Matrix<double, 6, 1> g_type;
    typedef Eigen::Matrix<double, 6, 1> dx_type;
};
} // namesapce algorithm_ns
  