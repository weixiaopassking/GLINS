/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/mapping/loop_closing/loop_closing.hpp"
// ros
#include <ros/ros.h>
#include <ros/package.h>
// log
#include <glog/logging.h>
// yaml
#include <yaml-cpp/yaml.h>

namespace multisensor_localization
{
    /**
     * @brief 回环算法 初始化配置
     * @note
     * @todo
     **/
    LoopClosing::LoopClosing()
    {
        std::string config_file_path = ros::package::getPath("multisensor_localization") +
                                       "/config/loop_closing.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        ConfigParam(config_node);
    }

    /**
     * @brief 回环算法 参数配置
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigParam(const YAML::Node &config_node)
    {
        extend_frame_num_ = config_node["extend_frame_num"].as<int>();
        loop_step_ = config_node["loop_step"].as<int>();
        diff_num_ = config_node["diff_num"].as<int>();
        detect_area_ = config_node["detect_area"].as<float>();
        fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();

        return true;
    }

    /**
     * @brief 回环算法 点云数据路径
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigDataPath(const YAML::Node &config_node)
    {
        std::string data_path = config_node["data_path"].as<std::string>();
        if (data_path == "./")
        {
            data_path = ros::package::getPath("multisensor_localization") + "/data";
        }
        else
        {
            LOG(ERROR) << std::endl
                       << "[yaml中的数据存放路径设置错误]" << std::endl;
            ROS_BREAK();
        }
        key_frame_path_=data_path+"key_frames";

        return true;
    }

    /**
     * @brief 回环算法 匹配方法
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        
    }

    /**
     * @brief 回环算法 配置滤波方法
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {

    }

} // namespace multisensor_localization
