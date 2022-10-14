/*
 * @Description: 位姿数据封装
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo 
 */

#include "../../include/sensor_data/pose_data.hpp"

namespace multisensor_localization
{
    /**
     * @brief 提取出旋转矩阵
     * @note 主要给imu数据用
     * @todo
     **/
    Eigen::Quaternionf PoseData::GetQuaternion()
    {
        Eigen::Quaternionf q;
        q=pose_.block<3, 3>(0, 0);
        return q;
    }

} // namespace multisensor_localization
