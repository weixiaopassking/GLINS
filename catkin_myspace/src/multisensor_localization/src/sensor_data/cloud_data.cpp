/*
 * @Description: 激光雷达数据封装
  * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 */

#include "../../include/sensor_data/cloud_data.hpp"

namespace multisensor_localization
{

    /**
     * @brief 激光雷达数据类型封装构造
     * @note 为cloud_ptr_分配内存空间
     * @todo
     **/
    CloudData::CloudData() : cloud_ptr_(new CLOUD())
    {
    }

}; // namespace multisensor_localization
