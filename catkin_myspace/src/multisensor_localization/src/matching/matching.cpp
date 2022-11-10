/*
 * @Description: 重定位算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-10
 */

/*
 * @Description: 匹配算法--初始化
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-08
 */

// relevent
#include "../../include/matching/matching.hpp"
// yaml库
#include <yaml-cpp/yaml.h>
// log
#include <glog/logging.h>
//ros
#include <ros/ros.h>
#include <ros/package.h>

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
        // ConfigDataPath();
        // ConfigRegistration();
        // ConfigFilter();
        // ConfigFilter();
        // ConfigBoxFilter();
        /*地图配置*/
        //初始化全局地图??
        //重置局部地图??
    }

}; //  namespace multisensor_localization