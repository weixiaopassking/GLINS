/**
*****************************************************************************
*  Copyright (C), 2023-2026,wengang.niu
*  @file    cloud_registration_interface.hpp
*  @brief  点云配准基类接口
*  @author  wengang.niu
*  @date    2023/7/26
*  @version v0.2
*  @ref github:gaoxiang12/slam_in_autonomous_driving
*  @note 已实现:point-point icp
****************************************************************************
*/
#ifndef _CLOUD_REGISTRATION_HPP
#define _CLOUD_REGISTRATION_HPP

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include "sensor_type/cloud_type.hpp"

#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点

#include "debug_utils.hpp"

namespace algorithm_ns
{

    class CloudRegistrationInterface
    {
      public:
        CloudRegistrationInterface(){};
        virtual void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr) = 0;
        virtual void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) = 0;
        virtual void SetGtTransform(const Sophus::SE3d &res_transform) = 0;
        virtual bool GetResTransform(Sophus::SE3d &init_transform) = 0;
        virtual ~CloudRegistrationInterface(){};//基类析构函数务必虚化以吊起继承类析构正确释放内存
    };
}
#endif //_CLOUD_REGISTRATION
