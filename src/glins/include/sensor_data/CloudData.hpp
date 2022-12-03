/*
 * @Description: cloud data type define
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-30
 * @Todo: add defination about colorful point cloud 
 */

#ifndef CLOUD_DATA_HPP_
#define CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glins
{

    class CloudData
    {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = pcl::PointCloud<pcl::PointXYZ>::Ptr;

    public:
    CloudData();
    private:
    double time_stamp=0.0;
    CLOUD_PTR cloud_ptr;
    }; // class cloud data
} // namespace glins

#endif