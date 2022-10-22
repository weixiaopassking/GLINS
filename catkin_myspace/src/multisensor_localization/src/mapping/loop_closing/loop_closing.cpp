/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/mapping/loop_closing/loop_closing.hpp"
// ndt
#include "../../../include/models/registration/ndt_registration.hpp"
// voxel
#include "../../../include/models/cloud_filter/voxel_filter.hpp"
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
        ConfigDataPath(config_node);
        ConfigRegistrationMethod(registration_ptr_, config_node);
        ConfigFilterMethod("map", map_filter_ptr_, config_node);
        ConfigFilterMethod("scan", scan_filter_ptr_, config_node);
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

        LOG(INFO) << "[extend_frame_num]" << std::endl
                  << extend_frame_num_ << std::endl;
        LOG(INFO) << "[loop_step]" << std::endl
                  << loop_step_ << std::endl;
        LOG(INFO) << "[diff_num]" << std::endl
                  << diff_num_ << std::endl;
        LOG(INFO) << "[detect_area]" << std::endl
                  << detect_area_ << std::endl;
        LOG(INFO) << "[fitness_score_limit]" << std::endl
                  << fitness_score_limit_ << std::endl;

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
        key_frame_path_ = data_path + "/key_frames";
        LOG(INFO) << "[key_frames 路径]" << std::endl
                  << key_frame_path_ << std::endl;
        return true;
    }

    /**
     * @brief 回环算法 匹配方法
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        std::string registration_method = config_node["registration_method"].as<std::string>();
        LOG(INFO) << "[registration_method]" << std::endl
                  << registration_method << std::endl;
        if (registration_method == "NDT")
        {
            registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
        }
        else
        {
            LOG(ERROR) << "[未能找到对应匹配方式]" << std::endl;
            ROS_BREAK();
        }
        return true;
    }

    /**
     * @brief 回环算法 配置滤波方法
     * @note
     * @todo
     **/
    bool LoopClosing::ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
        LOG(INFO) << "[闭环registration_method]" << std::endl
                  << filter_method << std::endl;
        if (filter_method == "voxel_filter")
        {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
        }
        else
        {
            LOG(ERROR) << "[无对应滤波] " << std::endl;
            ROS_BREAK();
        }
        return true;
    }

    /**
     * @brief 回环算法 更新
     * @note
     * @todo
     **/
    bool LoopClosing::Update(const KeyFrame key_frame, const KeyFrame key_gnss)
    {
        has_new_loop_pose_ = false;

        all_key_frames_.push_back(key_frame);
        all_key_gnss_.push_back(key_gnss);

        int key_frame_index=0;
        
        //查询最近帧

        //点云配准

        has_new_loop_pose_=true;
        return true;
        
    }

} // namespace multisensor_localization
