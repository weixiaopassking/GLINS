/*
 * @Description:不做滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/models/cloud_filter/no_filter.hpp"
// glog
#include <glog/logging.h>

namespace multisensor_localization
{
    /**
     * @brief 空构造
     * @note
     * @todo
     **/
    NoFilter::NoFilter()
    {
            LOG(INFO) << "[不滤波]" << std::endl;
    }
    /**
     * @brief 滤波
     * @note
     * @todo
     **/
    bool NoFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
        filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
        return true;
    }

} // namespace multisensor_localization