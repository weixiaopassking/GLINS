#ifndef _CLOUD_IO_HPP
#define _CLOUD_IO_HPP


#include "../../../../common/debug_info.hpp" //错误断言
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

    friend std::ostream &operator<<(std::ostream &o, const CloudIO &obj);

        ~CloudIO();

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_ptr;
};

#endif// _CLOUD_IO_HPP
