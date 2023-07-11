#ifndef _POINT_CLOUD_HPP
#define _POINT_CLOUD_HPP

#include <pcl/io/pcd_io.h>   //读写pcd
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //单点

#include <string>

class pointcloud_handle
{
  public:
    pointcloud_handle();
    pointcloud_handle(const std::string);
    ~pointcloud_handle();
};

#endif // _TEMPLATE_HPP