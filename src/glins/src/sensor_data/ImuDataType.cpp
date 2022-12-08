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
    /**
     * @brief  convert quaternion to rotation_matrix
     * @note
     * @todo
     **/
    Eigen::Matrix3f ImuDataType::OrientationToRotationMatrix()
    {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f rotation_matrix = q.matrix().cast<float>();
        return rotation_matrix;
    }

    /**
     * @brief  sync time to refer time
     * @note refer time sets point cloud time
     * @todo try slerp method
     **/
    bool ImuDataType::TimeSync(std::deque<ImuDataType> &unsynced_data_deque,
                               std::deque<ImuDataType> &synced_data_buff,
                               const double refer_time)
    {

    }
}; // namespace glins