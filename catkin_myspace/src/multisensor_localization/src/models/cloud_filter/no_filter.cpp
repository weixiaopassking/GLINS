/*
 * @Description:不做滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/models/cloud_filter/no_filter.hpp"

namespace multisensor_localization
{
    /**
     * @brief
     * @note
     * @todo
     **/
    NoFilter::NoFilter()
    {
    }
    /**
     * @brief
     * @note
     * @todo
     **/
    bool NoFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
    }

} // namespace multisensor_localization