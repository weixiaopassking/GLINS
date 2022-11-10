/*
 * @Description:其
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-17
 */

#ifndef BOX_FILTER_HPP_
#define BOX_FILTER_HPP_

//父类
#include "./cloud_filter_interface.hpp"
//数据类型
#include "../../sensor_data/cloud_data.hpp"
//pcl
#include <pcl/filters/crop_box.h>

namespace multisensor_localization
{

    class BoxFilter : public CloudFilterInterface
    {
    public:
        BoxFilter(YAML::Node node);
        BoxFilter() = default;
        //bool Filter(const CloudData::CLOUD_PTR &input_cloudc_ptr, const CloudData::CLOUD_PTR &filtered_cloud_ptr) override;

    private:
        pcl::CropBox<CloudData::POINT>pcl_box_filter;
        std::vector<float> origin_;
        std::vector<float> size_;
        std::vector<float> edge_;

    }; // class BoxFilter

} // namespace multisensor_localization

#endif