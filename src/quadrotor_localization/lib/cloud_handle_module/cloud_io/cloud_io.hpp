/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    cloud_io.hpp
*  @brief  点云的读写
*  @author  robotics gang
*  @date    2023/7/18
*  @version v0.1
*  @ref
****************************************************************************
*/

#include "../../../../common/error_assert.hpp" //错误断言
#include <pcl/io/pcd_io.h>                     //读写pcd
#include <pcl/point_cloud.h>                   //点云
#include <pcl/point_types.h>                   //点
#include <string>
#include <yaml-cpp/yaml.h>

class CloudIO
{
  public:
    CloudIO();
    bool LoadCloud(const std::string pcd_path); // 读取点云pcd
    bool LoadParam(const YAML::Node node);      // 读取参数

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloud();
    bool SaveCloud(); // 点云保存
    bool SaveParam(); // 参数保存
    ~CloudIO();

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_ptr;
};