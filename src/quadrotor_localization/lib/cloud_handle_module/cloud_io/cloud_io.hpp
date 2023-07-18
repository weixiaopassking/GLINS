/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    cloud_io.hpp
*  @brief  点云的文件交互
*  @author  robotics gang
*  @date    2023/7/11
*  @version v0.1
*  @ref  github.com/gaoxiang12/slam_in_autonomous_driving
****************************************************************************
*/

#include <string>

class CloudIO
{
  public:
    CloudIO() = delete;                  // 禁止无参构造
    bool LoadPcd(const std::string pcd_path); // 路径读取
    // LoadCloud(Yaml::Node)
    //  todo yaml配置
    bool SavePcd();

};