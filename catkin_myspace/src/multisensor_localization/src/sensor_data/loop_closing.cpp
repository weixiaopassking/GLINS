/*
 * @Description: 回环数据类型
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 * @Todo
 */

// relevent
#include "../../include/sensor_data/loop_closing.hpp"

namespace multisensor_localization
{
    /**
     * @brief 提取出旋转矩阵
     * @note 
     * @todo
     **/
    Eigen::Quaternionf LoopPose::GetQuaternion()
    {
        Eigen::Quaternionf q;
        q = pose.block<3, 3>(0, 0);

        return q;
    }

} // namespace multisensor_localization