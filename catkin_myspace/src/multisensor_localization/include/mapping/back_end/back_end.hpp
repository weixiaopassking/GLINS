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
#include "../../../include/sensor_data/pose_data.hpp"
#include "../../../include/sensor_data/key_frame.hpp"
// yaml参数库
#include <yaml-cpp/yaml.h>
// fstream
#include <fstream>
// c++
#include <deque>
// eigen
#include <Eigen/Dense>

namespace multisensor_localization
{
    class BackEnd
    {
    public:
        BackEnd();
        bool ForceOptimize();

        bool Update(const CloudData &cloud_data, const PoseData &laser_odom, const PoseData &gnss_odom);
        bool HasNewKeyFrame();  //主要是针对flow层的接口
        bool HasNewOptimized(); //主要是针对flow层的接口
        bool GetOptimizedKeyFrames(std::deque<KeyFrame> &key_frames_deque);
        void GetCurrentKeyFrame(KeyFrame &key_frame); //主要是针对flow层的接口

    private:
        /*参数配置*/
        bool ConfigFrame(const YAML::Node &config_node);
        bool ConfigGraphOptimizer(const YAML::Node &config_node);
        bool ConfigDataPath(const YAML::Node &config_node);

        bool IsNewKeyFrame(const CloudData &cloud_data, const PoseData &laser_odom);
        bool IsOptimized();
        bool AddNodeAndEdge(const PoseData &gnss_data);

        bool SaveTrajectory(std::ofstream &ofs, const Eigen::Matrix4f &pose);
        bool SaveOptimizedTrajectory();

    private:
        std::string key_frames_path_ = "";
        std::string trajectory_path_ = "";

        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;
        std::ofstream optimized_pose_ofs_;

        float key_frame_distance_ = 2.0;

        bool has_new_key_frame_ = false;
        bool has_new_optimized_ = false;

        KeyFrame current_key_frame_;
        std::deque<KeyFrame> key_frames_deque_;
        std::deque<Eigen::Matrix4f> optimized_pose_;

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
        }; //        class GraphOptimizerConfig

        GraphOptimizerConfig graph_optimizer_config_;
        int new_gnss_cnt_ = 0;
        int new_loop_cnt_ = 0;
        int new_key_frame_cnt_ = 0;
    };

}

#endif

// optimize_step_with_key_frame：当新到关键帧计数达到这个值时做一次优化
// optimize_step_with_gnss：当新产生产生的gnss观测技术达到这个值时做一次优化
// optimize_step_with_loop：当新得到闭环检测边的计数达到这个值时做一次优化
// 当这三个条件满足其中之一时，即做优化，并把三个计数均清零。