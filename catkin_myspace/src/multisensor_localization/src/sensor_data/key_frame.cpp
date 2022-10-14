/*
 * @Description: 关键帧数据封装
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo 
 */


//relevent
#include "../../include/sensor_data/key_frame.hpp"

namespace multisensor_localization
{
    /**
     * @brief 提取出旋转矩阵
     * @note
     * @todo
     **/
    Eigen::Quaternionf KeyFrame::GetQuaternion()
    {
        Eigen::Quaternionf q;
        q=pose_.block<3, 3>(0, 0);
        return q;
    }

} // namespace multisensor_localization
