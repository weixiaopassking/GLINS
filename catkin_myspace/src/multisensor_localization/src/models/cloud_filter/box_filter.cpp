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

    /**
     * @brief 全局地图切割出小地图
     * @note
     * @todo
     **/
    bool BoxFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr, CloudData::CLOUD_PTR &filtered_cloud_ptr)
    {
        filtered_cloud_ptr->clear();
        pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1e-4));
        pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1e-4));
        pcl_box_filter_.setInputCloud(input_cloud_ptr);
        pcl_box_filter_.filter(*filtered_cloud_ptr);
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
        CalculateBoxRange();
        return true;
    }

    /**
     * @brief 计算盒子尺寸
     * @note
     * @todo
     **/
    void BoxFilter::CalculateBoxRange()
    {
        for (size_t i = 0; i < origin_.size(); i++)
        {
            /*计算三轴 min max*/
            edge_.at(2 * i) = origin_.at(i) + size_.at(2 * i);
            edge_.at(2 * i + 1) = origin_.at(i) + size_.at(2 * i + 1);
        }
    }

    /**
     * @brief 输出子地图大小
     * @note
     * @todo
     **/
    std::vector<float> BoxFilter::GetEdge()
    {
        return edge_;
    }

} // namespace  multisensor_localization
