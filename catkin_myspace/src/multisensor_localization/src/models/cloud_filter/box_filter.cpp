
/*
 * @Description:不做滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/models/cloud_filter/box_filter.hpp"
// log
#include <glog/logging.h>

namespace multisensor_localization
{
    /**
     * @brief 设置滤波器尺寸--YAML参数方式
     * @note
     * @todo
     **/
    BoxFilter::BoxFilter(YAML::Node node)
    {
        /*参数重置*/
        size_.resize(6);
        edge_.resize(6);
        origin_.resize(3);
        /*加载滤波器参数*/
        size_ = node["box_filter_size"].as<std::vector<float>>();

        LOG(INFO) << "[box_filter_size]" << std::endl
                  << size_.at(0) << " "
                  << size_.at(1) << " "
                  << size_.at(2) << " "
                  << size_.at(3) << " "
                  << size_.at(4) << " "
                  << size_.at(5) << " " << std::endl;
    }

    /**
     * @brief 设置滤波器尺寸--函数参数形式
     * @note
     * @todo
     **/
    bool BoxFilter::SetSize(std::vector<float> size)
    {
        size_ = size;
        return true;
    }

    bool BoxFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {

        return true;
    }

    /**
     * @brief 设置起点
     * @note
     * @todo
     **/
    bool BoxFilter::SetOrigin(std::vector<float> origin)
    {
        origin_ = origin;
        return true;
    }

    /**
     * @brief 计算盒子尺寸
     * @note
     * @todo
     **/
    void BoxFilter::CalculateBoxRange()
    {

    }

} // namespace  multisensor_localization
