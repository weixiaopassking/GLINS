/*
 * @Description: 后端算法的具体实现
 * @Author: Ren Qian
 * @Date: 2022-10-11
 */

#ifndef BACK_END_HPP_
#define BACK_END_HPP_

//图优化器接口
#include "../../../include/models/graph_optimizer/graph_optimizer_interface.hpp"
//自定义传感器数据类型
#include "../../../include/sensor_data/cloud_data.hpp"
#include "../../../include/sensor_data/pose_data.hpp""
// yaml参数库
#include <yaml-cpp/yaml.h>
//fstream
#include <fstream>

namespace multisensor_localization
{
    class BackEnd
    {
    public:
        BackEnd();

        bool Update(const  CloudData&cloud_data,const PoseData&laser_odom);

    private:
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigGraphOptimizer(const YAML::Node &config_node);
        bool ConfigDataPath(const YAML::Node &config_node);

    private:
        std::string key_frames_path_ = "";
        std::string trajectory_path_ = "";

        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;

        float key_frame_distance_ = 2.0;

        std::shared_ptr<GraphOptimizerInterface> graph_optimizer_ptr_;

        class GraphOptimizerConfig
        {
        public:
            GraphOptimizerConfig()
            {
                odom_noise_.resize(6);
                close_loop_noise_.resize(6);
                gnss_nosie_.resize(3);
            }

        public:
            bool use_gnss = true;
            bool use_close_loop = true;

            Eigen::VectorXd odom_noise_;
            Eigen::VectorXd close_loop_noise_;
            Eigen::VectorXd gnss_nosie_;

            int key_frame_optimize_step_ = 100;
            int gnss_optimize_step_ = 100;
            int close_loop_optimize_step_ = 100;
        };
        GraphOptimizerConfig graph_optimizer_config_;
        int new_gnss_cnt_ = 0;
        int new_loop_cnt_ = 0;
        int new_key_frame_cnt_ = 0;
    };

}

#endif