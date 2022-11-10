
/*
 * @Description:不做滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/models/cloud_filter/box_filter.hpp"

namespace multisensor_localization
{

    BoxFilter::BoxFilter(YAML::Node node)
    {
        /*参数重置*/
        size_.resize(6);
        edge_.resize(6);
        origin_.resize(6);
        /*加载滤波器参数*/
        size_ = node["box_filter_size"].as<std::vector<float>>();
    }
    /**
     * @brief
     * @note
     * @todo
     **/
    // bool BoxFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, const CloudData::CLOUD_PTR &filtered_cloud_ptr)
    // {
    //     return true;
    // }

} // namespace  multisensor_localization
