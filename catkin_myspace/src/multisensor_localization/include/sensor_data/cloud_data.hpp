/*
 * @Description: 激光雷达数据封装
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 */

#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace multisensor_localization
{
  class CloudData
  {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<pcl::PointXYZ>;
    using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  public:
    CloudData();
    
  public:
    double time_stamp_= 0.0;
    CLOUD_PTR cloud_ptr_;
  };
} // namespace multisensor_localization 

#endif