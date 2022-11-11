/*
 * @Description: 重定位算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-08
 */

#ifndef MATCHING_HPP_
#define MATCHING_HPP_

// c++库
#include <string>
//自定义消息类型
#include "../sensor_data/cloud_data.hpp"
// models
#include "../models/registration/registration_interface.hpp"
#include "../models/cloud_filter/cloud_filter_interface.hpp"
#include "../../include/models/cloud_filter/box_filter.hpp"

namespace multisensor_localization
{
    class Matching
    {
    public:
        Matching();

    private:
        /*参数配置*/
        bool ConfigDataPath(const YAML::Node &config_node);
        bool ConfigRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool ConfigFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool ConfigBoxFilter(const YAML::Node &config_node);
        /*地图配置*/
        bool LoadGlobalMap();
        bool ResetLocalMap(float x,float y,float z);

    private:
        std::string map_path_ = "";

         std::shared_ptr<BoxFilter> box_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> current_scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;

        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;

        bool has_new_global_map_=false;
        bool has_new_local_map_=false;

    }; // class Matching

} // namespace multisensor_localization

#endif