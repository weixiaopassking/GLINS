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

#include "yaml-cpp/yaml.h"
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点
#include "../../../../common/error_assert.hpp"//错误断言

#include <string>

class CloudIO
{
  public:
    CloudIO();
    bool LoadCloud(const std::string pcd_path);      
    bool LoadParam(const YAML::Node node);       

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloud(); 
    bool SaveCloud();                                // 点云储存
    bool SaveParam();
    ~CloudIO();

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_ptr;
};