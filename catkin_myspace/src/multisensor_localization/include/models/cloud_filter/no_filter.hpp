/*
 * @Description:不滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#ifndef NO_FILTER_HPP_
#define NO_FILTER_HPP_

#include "./cloud_filter_interface.hpp"

namespace multisensor_localization
{
    class NoFilter : public CloudFilterInterface
    {
    public:
    NoFilter();
    bool Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr);
    

    }; // class NoFilter

} // namespace multisensor_localization

#endif