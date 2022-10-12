/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */

//后端算法
#include "../../../include/mapping/back_end/back_end.hpp"
// yaml库
#include <yaml-cpp/yaml.h>
// log
#include <glog/logging.h>
// ros库
#include <ros/ros.h>
#include <ros/package.h>
// debug tool
#include "../../../include/tools/color_terminal.hpp"
// g2o优化算法
#include "../../../include/models/graph_optimizer/g2o/g2o_optimizer.hpp"
// tools
#include "../../../include/tools/file_manager.hpp"

namespace multisensor_localization
{
    /**
     * @brief 后端算法初始化
     * @note 读取yaml参数并配置 关键帧、图优化方式、点云存放地址
     * @todo
     **/
    BackEnd::BackEnd()
    {
        /*参数文件读取*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/back_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
        // DebugTools::Debug_Info("参数配置中");
        ConfigFrame(config_node);
        ConfigGraphOptimizer(config_node);
        ConfigDataPath(config_node);
         
    }

    /**
     * @brief 关键帧参数配置
     * @note 选取关键帧的距离
     * @todo
     **/
    bool BackEnd::ConfigFrame(const YAML::Node &config_node)
    {
        key_frame_distance_ = config_node["key_frame_distance"].as<float>();
        LOG(INFO) << "[关键帧距离] " << std::endl
                  << key_frame_distance_ << std::endl;
        return true;
    }

    /**
     * @brief   图优化方法设置
     * @note 目前仅支持g2o
     * @todo
     **/
    bool BackEnd::ConfigGraphOptimizer(const YAML::Node &config_node)
    {
        /*选定优化器*/
        std::string graph_optimizer_type = config_node["graph_optimizer_type"].as<std::string>();
        if (graph_optimizer_type == "g2o")
        {
            graph_optimizer_ptr_ = std::make_shared<G2oOptimizer>("lm_var");
            LOG(INFO) << "[图优化方法]" << std::endl
                      << graph_optimizer_type << std::endl;
        }
        else
        {
            LOG(ERROR) << graph_optimizer_type << "[未能找到对应的优化方法]" << std::endl;
            ROS_BREAK();
        }

        /*优化是否考虑闭环、回环*/
        graph_optimizer_config_.use_gnss = config_node["use_gnss"].as<bool>();
        graph_optimizer_config_.use_close_loop = config_node["use_close_loop"].as<bool>();
        LOG(INFO) << "[use_gnss]" << std::endl
                  << graph_optimizer_config_.use_gnss << std::endl;
        LOG(INFO) << "[use_close_loop]" << std::endl
                  << graph_optimizer_config_.use_close_loop << std::endl;

        /*优化间隔*/
        graph_optimizer_config_.key_frame_optimize_step_ = config_node["key_frame_optimize_step"].as<int>();
        graph_optimizer_config_.gnss_optimize_step_ = config_node["gnss_optimize_step"].as<int>();
        graph_optimizer_config_.close_loop_optimize_step_ = config_node["close_loop_optimize_step"].as<int>();
        LOG(INFO) << "[key_frame_optimize_step]" << std::endl
                  << graph_optimizer_config_.key_frame_optimize_step_ << std::endl;
        LOG(INFO) << "[gnss_optimize_step]" << std::endl
                  << graph_optimizer_config_.gnss_optimize_step_ << std::endl;
        LOG(INFO) << "[close_loop_optimize_step]" << std::endl
                  << graph_optimizer_config_.close_loop_optimize_step_
                  << std::endl;

        /* 数据噪声*/
        graph_optimizer_config_.close_loop_noise_ = Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor>>(config_node[graph_optimizer_type + "_param"]["close_loop_noise"].as<std::vector<double>>().data());
        graph_optimizer_config_.odom_noise_ = Eigen::Map<Eigen::Matrix<double, 6, 1, Eigen::ColMajor>>(config_node[graph_optimizer_type + "_param"]["odom_noise"].as<std::vector<double>>().data());
        graph_optimizer_config_.gnss_nosie_ = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(config_node[graph_optimizer_type + "_param"]["odom_noise"].as<std::vector<double>>().data());
        LOG(INFO) << "[close_loop_noise]" << std::endl
                  << graph_optimizer_config_.close_loop_noise_
                  << std::endl;
        LOG(INFO) << "[odom_noise]" << std::endl
                  << graph_optimizer_config_.odom_noise_
                  << std::endl;
        LOG(INFO) << "[gnss_nosie]" << std::endl
                  << graph_optimizer_config_.gnss_nosie_
                  << std::endl;

        return true;
    }

    /**
     * @brief   配置数据保存路径
     * @note
     * @todo 创建文件时候用文件管理器
     **/
    bool BackEnd::ConfigDataPath(const YAML::Node &config_node)
    {
        /*读参数据存放路径*/
        std::string data_path = config_node["data_path"].as<std::string>();
        if (data_path == "./")
        {
            data_path = ros::package::getPath("multisensor_localization") + "/data";
        }
        else
        {
            LOG(ERROR) << "[yaml中的数据存放路径设置错误]" << std::endl;
        }
        /*创建文件夹data*/
        if (!FileManager::CreateDirectory(data_path))
        {
            LOG(ERROR) << "[创建data文件夹失败]" << std::endl;
            return false;
        }
        LOG(INFO) << "[创建data文件夹成功]" << std::endl;
        /*创建文件夹data/key_frames*/
        key_frames_path_ = data_path + "/key_frames";
        if (!FileManager::CreateDirectory(key_frames_path_))
        {
            LOG(ERROR) << "[创建data/key_frames文件夹失败]" << std::endl;
            return false;
        }
        LOG(INFO) << "[创建data/key_frames文件夹成功]" << std::endl;
        /*创建文件夹data/trajectory*/
        trajectory_path_ = data_path + "/trajectory";
        if (!FileManager::CreateDirectory(trajectory_path_))
        {
            LOG(ERROR) << "[创建data/trajectory文件夹成功]" << std::endl;
            return false;
        }
        LOG(INFO) << "[创建data/trajectory文件夹成功]" << std::endl;

        /*创建文件data/trajectory/ground_truth.txt*/

        if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        {
            LOG(ERROR) << "[创建ground_truth.txt文件失败]" << std::endl;
            return false;
        }
        LOG(INFO) << "[创建ground_truth.txt文件成功]" << std::endl;

        /*创建文件data/trajectory/laser_odom.txt*/
        if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        {
            LOG(ERROR) << "[创建laser_odom.文件失败]" << std::endl;
            return false;
        }
        LOG(INFO) << "[创建laser_odom文件成功]" << std::endl;

        return true;
    }

}
