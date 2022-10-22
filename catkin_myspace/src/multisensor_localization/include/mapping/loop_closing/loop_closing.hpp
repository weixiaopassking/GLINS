/*
 * @Description: 回环检测算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

#ifndef LOOP_CLOSING_HPP_
#define LOOP_CLOSING_HPP_

// relevent
#include <deque>
// sensor_data
#include "../../sensor_data/key_frame.hpp"
#include "../../sensor_data/loop_pose.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// models
#include "../../models/registration/registration_interface.hpp"
#include "../../models/cloud_filter/cloud_filter_interface.hpp"

namespace multisensor_localization
{
    class LoopClosing
    {
    public:
        LoopClosing();
        bool Update(const KeyFrame key_frame, const KeyFrame key_gnss);
        bool HasNewLoopPose();
        LoopPose &GetCurrentLoopPose();

    private:
        bool ConfigParam(const YAML::Node &config_node);
        bool ConfigDataPath(const YAML::Node &config_node);
        bool ConfigRegistrationMethod(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);
        bool ConfigFilterMethod(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node);

    private:
        std::string key_frame_path_ = "";
        int extend_frame_num_ = 3;
        int loop_step_ = 10;
        int diff_num_ = 100;
        float detect_area_= 10.0;
        float fitness_score_limit_ = 2.0;
        /*models*/
        std::shared_ptr<CloudFilterInterface> scan_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> map_filter_ptr_;
        std::shared_ptr<RegistrationInterface> registration_ptr_;
        /*数据队列*/
        std::deque<KeyFrame> all_key_frames_;
        std::deque<KeyFrame> all_key_gnss_;

        LoopPose current_loop_pose_;
        bool has_new_loop_pose_=false;
    }; // class LoopClosing

} // namespace multisensor_localization

#endif