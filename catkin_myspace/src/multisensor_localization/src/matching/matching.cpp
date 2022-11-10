/*
 * @Description: 重定位算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-10
 */

// relevent
#include "../../include/matching/matching.hpp"
// yaml库
#include <yaml-cpp/yaml.h>
// log
#include <glog/logging.h>
// ros
#include <ros/ros.h>
#include <ros/package.h>
// modles
#include "../../include/models/registration/ndt_registration.hpp"
#include "../../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{
    /**
     * @brief 重定位算法--初始化
     * @note
     * @todo
     **/
    Matching::Matching()
    {
        /*地图指针分配内存空间*/
        current_scan_ptr_.reset(new CloudData::CLOUD());
        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());

        /*参数文件读取*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") +
                                       "/config/matching.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
        ConfigDataPath(config_node);
        ConfigRegistration(registration_ptr_, config_node);
        ConfigFilter("global_map", global_map_filter_ptr_, config_node);
        ConfigFilter("local_map", local_map_filter_ptr_, config_node);
        ConfigFilter("current_scan", current_scan_filter_ptr_, config_node);
        // ConfigFilter();
        // ConfigBoxFilter();
        /*地图配置*/
        //初始化全局地图??
        //重置局部地图??
    }

    /**
     * @brief 重定位算法--配置数据存放
     * @note
     * @todo
     **/
    bool Matching::ConfigDataPath(const YAML::Node &config_node)
    {
        map_path_ = config_node["map_path"].as<std::string>();
        LOG(INFO) << "[map_path] " << std::endl
                  << map_path_ << std::endl;
        return true;
    }

    /**
     * @brief 重定位算法--地图匹配方式
     * @note
     * @todo
     **/
    bool Matching::ConfigRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        std::string registration_method = config_node["registration_method"].as<std::string>();
        if (registration_method == "NDT")
        {
            registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method] " << std::endl
                      << registration_method << std::endl;
            return true;
        }
        else
        {
            LOG(ERROR) << "[no matching  registration method] " << std::endl;
            ROS_BREAK();
        }
    }

    /**
     * @brief 重定位算法--地图滤波
     * @note
     * @todo
     **/
    bool Matching::ConfigFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
        if (filter_method == "voxel_filter")
        {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
            LOG(INFO) << "[filter_method] " << std::endl
                      << filter_method << std::endl;
            return true;
        }
        else
        {
            LOG(ERROR) << "[no matching filter method] " << std::endl;
            ROS_BREAK();
        }
    }

    bool ConfigBoxFilter(const YAML::Node &config_node)
    {
        
    }

}; //  namespace multisensor_localization