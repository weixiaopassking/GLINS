/*
 * @Description:可视化任务管理器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-31
 */

// relevent
#include "../../../include/mapping/viewer/viewer.hpp"
// ros
#include <ros/ros.h>
#include <ros/package.h>
// log
#include <glog/logging.h>
// tools
#include "../../../include/tools/file_manager.hpp"
// voxel_filter
#include "../../../include/models/cloud_filter/voxel_filter.hpp"

namespace multisensor_localization
{
  /**
   * @brief 可视化流程初始化
   * @note 参数配置
   * @todo
   **/
  Viewer::Viewer()
  {
    std::string config_file_path = ros::package::getPath("multisensor_localization") + "/config/viewer.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    ConfigFrame(config_node);
    ConfigDataPath(config_node);
    ConfigFilterMethod("frame", frame_filter_ptr_, config_node);
    ConfigFilterMethod("local_map", local_map_filter_ptr_, config_node);
    ConfigFilterMethod("global_map", global_map_filter_ptr_, config_node);
  }

  /**
   * @brief 参数配置--关键帧
   * @note
   * @todo
   **/
  bool Viewer::ConfigFrame(const YAML::Node &config_node)
  {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    LOG(INFO) << "[local_frame_num] " << std::endl
              << local_frame_num_ << std::endl;
    return true;
  }

  /**
   * @brief 参数配置--滤波方法
   * @note
   * @todo
   **/
  bool Viewer::ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
  {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();

    if (filter_mothod == "voxel_filter")
    {
      filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
      LOG(INFO) << "[filter_mothod]" << std::endl
                << filter_mothod << std::endl;
    }
    else
    {
      LOG(ERROR)
          << "[无对应滤波方法]" << std::endl;
      ROS_BREAK();
    }
    return true;
  }

  /**
   * @brief 参数配置--数据路径
   * @note
   * @todo
   **/
  bool Viewer::ConfigDataPath(const YAML::Node &config_node)
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
      ROS_BREAK();
    }
    key_frames_path_ = data_path + "/key_frames";
    map_path_ = data_path + "/map";

    if (!FileManager::CreateDirectory(map_path_))
    {
      LOG(ERROR) << "[地图文件夹创建失败]" << std::endl;
      ROS_BREAK();
    }
    else
    {
      LOG(INFO) << "[地图文件夹创建成功]" << std::endl;
    }
    return true;
  }

  /**
   * @brief 更新优化帧
   * @note
   * @todo
   **/
  bool Viewer::UpdateOptimizedKeyFrames(std::deque<KeyFrame> &optimized_key_frames)
  {
    has_global_map_ = false;
    if (optimized_key_frames.size() > 0)
    {
optimized_key_frames
    }

  }

  /**
   * @brief 更新当前帧
   * @note
   * @todo
   **/
  bool Viewer::UpdateCurrentKeyFrame(std::deque<KeyFrame> &new_key_frames, PoseData transformed_data,
                                     CloudData cloud_data)
  {
  }

} // namespace multisensor_localization
