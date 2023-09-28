/*
 * @Description: definition for point cloud

 * @Author: niu_wengang@163.com
 * @Date: 2023-08-07
 * @Modified: 2023-08-07
 */

#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#define PCL_NO_PRECOMPILE
#include <pcl/pcl_macros.h>

struct EIGEN_ALIGN16 MyPointType
{
    PCL_ADD_POINT4D;   // add xyz
    PCL_ADD_RGB;       // add RGB
    PCL_ADD_INTENSITY; // add Intensity
    float time_offset; // lidar time offset
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,(float, x, x)\
(float, y, y)\
(float, z, z)\
(unsigned int, b, b)\
(unsigned int, g, g)\
(unsigned int, r, r)\
(unsigned, a, a)\
(float, intensity,intensity)\
(float, time_offset, time_offset))

namespace msg
{
class CloudData
{
  public:
    using POINT = MyPointType;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData();
    ~CloudData();
    void Undistort();//! todo 
    long double timestamp_ns;
    CLOUD_PTR cloud_ptr;
};
} // namespace msg

#endif // CLOUD_DATA_HPP_