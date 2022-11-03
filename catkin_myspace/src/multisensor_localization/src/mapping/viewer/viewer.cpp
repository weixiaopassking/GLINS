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
// pcl
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

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

  bool Viewer::SaveMap()
  {
    if (optimized_key_frames_.size() == 0)
      return false;
    /*创建全局地图指针*/
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    /*加载拼接地图*/
    for (size_t i = 0; i < optimized_key_frames_.size(); i++)
    {
      std::string map_file_path = key_frames_path_ + "/key_frame_" + std::to_string(optimized_key_frames_.at(i).index_) + ".pcd";
      CloudData::CLOUD_PTR cloud_temp_ptr(new CloudData::CLOUD());
      pcl::io::savePCDFileBinary(map_file_path, *cloud_temp_ptr);
      pcl::transformPointCloud(*cloud_temp_ptr, *cloud_temp_ptr, optimized_key_frames_.at(i).pose_);
      *global_map_ptr += *cloud_temp_ptr;
    }
    /*保存全局地图的PCD文件*/
    std::string map_file_path = map_path_ + "/map.pcd";
    std::cout << map_file_path << std::endl;
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "[map_pcd save path]" << std::endl
              << map_path_ << std::endl;
  }

} // namespace multisensor_localization
