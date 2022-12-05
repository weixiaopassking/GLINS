/*
 * @Description: gnss fix data type define
 * @Function:
 * @Author: niu_wengang@163.com
 * @Version : v1.0
 * @Date: 2022-11-22
 * @Note: Using the WGS 84 reference ellipsoid
 */

// relevent
#include "../../include/sensor_data/ImuDataType.hpp"

namespace glins
{
    Eigen::Matrix3f ImuDataType::OrientationToRotationMatrix()
    {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f rotation_matrix = q.matrix().cast<float>();
        return rotation_matrix;
    }
}; // namespace glins