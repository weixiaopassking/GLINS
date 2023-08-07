/*
 * @Description: definition for point cloud
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 */

#ifndef _CLOUD_DATA_HPP
#define _CLOUD_DATA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace data_ns
{
class CloudData
{
  public:

    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData();

  public:
    double _time_stamp = 0.0;//sec use unix time
    CLOUD_PTR _cloud_ptr;
    
}; // class CloudData
} // namespace data_ns

#endif //_CLOUD_DATA_HPP