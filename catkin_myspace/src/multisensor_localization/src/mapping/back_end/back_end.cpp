/*
 * @Description: 后端算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-19
 */

// relevent
#include "../../../include/mapping/back_end/back_end.hpp"
// yaml库
#include <yaml-cpp/yaml.h>
// log
#include <glog/logging.h>
// ros库
#include <ros/ros.h>
#include <ros/package.h>
//  tools
#include "../../../include/tools/color_terminal.hpp"
#include "../../../include/tools/file_manager.hpp"
// g2o
#include "../../../include/models/graph_optimizer/g2o/g2o_optimizer.hpp"
// pcl
#include <pcl/io/pcd_io.h>

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
        std::string config_file_path = ros::package::getPath("multisensor_localization") +
                                       "/config/back_end.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
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
        LOG(INFO) << "[key_frame_distance] " << std::endl
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
            LOG(INFO) << "[graph_optimizer_type]" << std::endl
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

        /* 数据噪声 信息*/
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
     * @note   过程中失败不返回直接ROS_BREAK()
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
            LOG(ERROR) << std::endl
                       << "[yaml中的数据存放路径设置错误]" << std::endl;
            ROS_BREAK();
        }

        /*创建文件夹data*/
        if (!FileManager::CreateDirectory(data_path))
        {
            LOG(ERROR) << "[创建data文件夹失败]" << std::endl;
            ROS_BREAK();
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
            LOG(ERROR) << "[创建data/trajectory文件夹失败]" << std::endl;
            ROS_BREAK();
        }
        LOG(INFO) << "[创建data/trajectory文件夹成功]" << std::endl;

        /*创建文件data/trajectory/ground_truth.txt*/
        if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        {
            LOG(ERROR) << "[创建ground_truth.txt文件失败]" << std::endl;
            ROS_BREAK();
        }
        LOG(INFO) << "[创建ground_truth.txt文件成功]" << std::endl;

        /*创建文件data/trajectory/laser_odom.txt*/
        if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        {
            LOG(ERROR) << "[创建laser_odom文件失败]" << std::endl;
            ROS_BREAK();
        }
        LOG(INFO) << "[创建laser_odom文件成功]" << std::endl;

        return true;
    }

    /**
     * @brief   后端轨迹优化更新(核心)
     * @note
     * @todo
     **/
    bool BackEnd::Update(const CloudData &cloud_data, const PoseData &laser_odom, const PoseData &gnss_odom)
    {
        /*重置标志位*/
        has_new_key_frame_ = true;
        has_new_optimized_ = true;
        /*保存轨迹*/ // todo 暂鸽咕咕咕
                     // SaveTrajectory();

        /*判断是否是关键帧*/
        if (IsNewKeyFrame(cloud_data, laser_odom))
        {

            /*添加顶点和边*/
            AddNodeAndEdge(gnss_odom);
            //判断是否优化
            IsOptimized();

            ColorTerminal::ColorConcreteDebug("判定为关键帧");
            std::cout << new_gnss_cnt_ << std::endl;
        }
        return true;
    }

    /**
     * @brief   是否有新关键帧生成
     * @note  根据欧式距离判断是否有新的关键帧生成，若有则写入硬盘并加入关键帧队列，再赋予当前关键帧新数值
     * @todo
     **/
    bool BackEnd::IsNewKeyFrame(const CloudData &cloud_data, const PoseData &laser_odom)
    {
        /*缓存历史帧*/
        static Eigen::Matrix4f last_key_pose = laser_odom.pose_;
        /*若是第一帧 那它即是关键帧*/
        if (key_frames_deque_.size() == 0)
        {
            has_new_key_frame_ = true;
            last_key_pose = laser_odom.pose_;
        }
        /*根据距离判断是否需要生成关键帧 只有一帧时当然不需要*/
        if (fabs(laser_odom.pose_(0, 3) - last_key_pose(0, 3)) +
                fabs(laser_odom.pose_(1, 3) - last_key_pose(1, 3)) +
                fabs(laser_odom.pose_(2, 3) - last_key_pose(2, 3)) >
            key_frame_distance_)
        {
            has_new_key_frame_ = true;
            last_key_pose = laser_odom.pose_;
        }

        /*如果确实符合关键帧的条件*/
        if (has_new_key_frame_)
        {
            /*关键帧点云存储到硬盘中*/
            std::string pcd_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_path, *(cloud_data.cloud_ptr_));

            /*当前激光里程计数据(已转到gnss坐标系下)创建临时关键帧*/
            KeyFrame key_frame;
            key_frame.time_stamp_ = laser_odom.time_stamp_;
            key_frame.index_ = (unsigned int)key_frames_deque_.size();
            key_frame.pose_ = laser_odom.pose_;
            /*加入关键帧队列*/
            key_frames_deque_.push_back(key_frame);
            /*同时赋值给当前关键帧*/
            current_key_frame_ = key_frame;
            return has_new_key_frame_;
        }
        /*返回新帧生成的标志*/
        return has_new_key_frame_;
    }

    /**
     * @brief   添加顶点和边
     * @note
     * 【顶点:】激光里程计关键帧的位姿(已转换到gnss坐标系下)
     *  【边】激光里程计关键帧的相对位姿(已转换到gnss坐标系下)
     *  【先验边】gnss东北天相对位置
     *注意：先加顶点后加边
     * @todo
     **/
    bool BackEnd::AddNodeAndEdge(const PoseData &gnss_data)
    {
        Eigen::Isometry3d transform; //也就是是T矩阵
        /*【添加顶点】雷达里程计关键帧的位姿(非固定)*/
        transform.matrix() = current_key_frame_.pose_.cast<double>();
        graph_optimizer_ptr_->AddSe3Node(transform, false);
        new_key_frame_cnt_++;

        /*【添加边】激光里程计帧间约束*/
        /*缓存历史数据*/
        static KeyFrame last_key_frame = current_key_frame_;
        int node_num = graph_optimizer_ptr_->GetNodeNum(); //获得顶点数
        if (node_num >= 2)                                 // 2个及2个以上的节点数时才有条件添加帧间约束的边
        {
            Eigen::Matrix4f last_to_now = last_key_frame.pose_.inverse() * current_key_frame_.pose_;
            transform.matrix() = last_to_now.cast<double>();
            graph_optimizer_ptr_->AddSe3Edge(node_num - 2, node_num - 1, transform, graph_optimizer_config_.odom_noise_);
        }
        last_key_frame = current_key_frame_; //缓存历史数据

        /*【添加边】gnss先验信息 三轴约束*/
        if (graph_optimizer_config_.use_gnss)
        {
            Eigen::Vector3d xyz(static_cast<double>(gnss_data.pose_(0, 3)),
                                static_cast<double>(gnss_data.pose_(1, 3)),
                                static_cast<double>(gnss_data.pose_(2, 3)));
            graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num - 1, xyz, graph_optimizer_config_.gnss_nosie_);
            new_gnss_cnt_++;
        }
        return true;
    }

    /**
     * @brief   是否执行优化
     * @note
     * @todo
     **/
    bool BackEnd::IsOptimized()
    {
        /*局部标志位*/
        bool is_need_optimizd = false;

        /*根据预设频率判断是否需要优化 三者满足其一即可*/
        if (new_gnss_cnt_ >= graph_optimizer_config_.gnss_optimize_step_)
        {
            is_need_optimizd = true;
        }
        if (new_loop_cnt_ >= graph_optimizer_config_.close_loop_optimize_step_)
        {
            is_need_optimizd = true; //不开启new_loop_cnt_不会计数据也就不会到这里
        }
        if (new_key_frame_cnt_ >= graph_optimizer_config_.key_frame_optimize_step_)
        {
            is_need_optimizd = true;
        }
        /*均未到预设频率则退出*/
        if (!is_need_optimizd)
        {
            return false;
        }
        /*到预设其中任何一个频率，所有计数清空*/
        new_gnss_cnt_ = 0;
        new_loop_cnt_ = 0;
        new_key_frame_cnt_ = 0;
        /*执行优化器，并刷是否有新优化产生的标志位*/
        if (graph_optimizer_ptr_->Optimize())
        {
            has_new_optimized_ = true;
            ColorTerminal::ColorConcreteDebug("执行优化");
        }
        return true;
    }

    /**
     * @brief   取出优化结果
     * @note 信息发布时候用
     * @todo
     **/
    bool BackEnd ::GetOptimizedKeyFrames(std::deque<KeyFrame> &key_frames_deque)
    {
        if (graph_optimizer_ptr_->GetNodeNum() > 0)
        {
            std::deque<Eigen::Matrix4f> optimized_pose;
            graph_optimizer_ptr_->GetOptimizedPose(optimized_pose);
            KeyFrame key_frame;
            size_t num = optimized_pose.size();
            for (size_t i = 0; i < num; i++)
            {
                key_frame.pose_ = optimized_pose.at(i);
                key_frame.index_ = (unsigned int)i;
                key_frames_deque.push_back(key_frame);
            }
        }
    }

    /**
     * @brief   是否产生关键帧
     * @note 从back_end取到back_end_flow
     * @todo
     **/
    bool BackEnd::HasNewKeyFrame()
    {
        return has_new_key_frame_;
    }

    /**
     * @brief   是否有新优化结果产生
     * @note 从back_end取到back_end_flow
     * @todo
     **/
    bool BackEnd::HasNewOptimized()
    {
        return has_new_optimized_;
    }

    /**
     * @brief   是否有新优化结果产生
     * @note 从back_end取到back_end_flow
     * @todo
     **/
    void BackEnd::GetCurrentKeyFrame(KeyFrame &key_frame)
    {
        key_frame = current_key_frame_;
    }

} // namespace multisensor_localization
