/*
 * @Description: Visualization algorithm
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-2
 */

#ifndef VIEWER_HPP_
#define VIEWER_HPP_

// yaml cpp
#include <yaml-cpp/yaml.h>
// cloud filter  interface
#include "../../models/cloud_filter/voxel_filter.hpp"
//数据类型
#include "../../sensor_data/key_frame.hpp"
#include "../../sensor_data/pose_data.hpp"
// eigen
#include <Eigen/Dense>

namespace multisensor_localization
{

    class Viewer
    {
    public:
        Viewer();
        bool SaveMap();
           std::deque<KeyFrame> optimized_key_frames_;

    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);
        bool ConfigDataPath(const YAML::Node &config_node);

    private:
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

  

        std::string data_path = "";
        std::string key_frames_path_ = "";
        std::string map_path_ = "";
        int local_frame_num_ = 20;
    };

} // namespace multisensor_localization

#endif
