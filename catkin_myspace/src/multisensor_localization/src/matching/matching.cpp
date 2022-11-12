/*
 * @Description: 重定位算法
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-10
 */

// relevent
#include "../../include/matching/matching.hpp"
// yaml库
#include <yaml-cpp/yaml.h>
// log
#include <glog/logging.h>
// ros
#include <ros/ros.h>
#include <ros/package.h>
// modles
#include "../../include/models/registration/ndt_registration.hpp"
#include "../../include/models/cloud_filter/voxel_filter.hpp"
// pcl
#include <pcl/io/pcd_io.h>

namespace multisensor_localization
{
    /**
     * @brief 重定位算法--初始化
     * @note
     * @todo
     **/
    Matching::Matching()
    {
        /*地图指针分配内存空间*/
        current_scan_ptr_.reset(new CloudData::CLOUD());
        local_map_ptr_.reset(new CloudData::CLOUD());
        global_map_ptr_.reset(new CloudData::CLOUD());

        /*参数文件读取*/
        std::string config_file_path = ros::package::getPath("multisensor_localization") +
                                       "/config/matching.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);
        /*参数配置*/
        ConfigDataPath(config_node);
        ConfigRegistration(registration_ptr_, config_node);
        ConfigFilter("global_map", global_map_filter_ptr_, config_node);
        ConfigFilter("local_map", local_map_filter_ptr_, config_node);
        ConfigFilter("current_scan", current_scan_filter_ptr_, config_node);
        ConfigBoxFilter(config_node);
        /*地图配置*/
        //初始化全局地图
        LoadGlobalMap();
        //重置局部地图
        ResetLocalMap(0.0, 0.0, 0.0);
    }

    /**
     * @brief 重定位算法--配置数据存放
     * @note
     * @todo
     **/
    bool Matching::ConfigDataPath(const YAML::Node &config_node)
    {
        map_path_ = config_node["map_path"].as<std::string>();
        LOG(INFO) << "[map_path] " << std::endl
                  << map_path_ << std::endl;
        return true;
    }

    /**
     * @brief 重定位算法--地图匹配方式
     * @note
     * @todo
     **/
    bool Matching::ConfigRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node)
    {
        std::string registration_method = config_node["registration_method"].as<std::string>();
        if (registration_method == "NDT")
        {
            registration_ptr = std::make_shared<NdtRegistration>(config_node[registration_method]);
            LOG(INFO) << "[registration_method] " << std::endl
                      << registration_method << std::endl;
            return true;
        }
        else
        {
            LOG(ERROR) << "[no matching  registration method] " << std::endl;
            ROS_BREAK();
        }
    }

    /**
     * @brief 重定位算法--地图滤波
     * @note
     * @todo
     **/
    bool Matching::ConfigFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr, const YAML::Node &config_node)
    {
        std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
        if (filter_method == "voxel_filter")
        {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
            LOG(INFO) << "[" << filter_user << "--filter_method] " << std::endl
                      << filter_method << std::endl;
            return true;
        }
        else
        {
            LOG(ERROR) << "[no matching filter method] " << std::endl;
            ROS_BREAK();
        }
    }

    /**
     * @brief 重定位算法--地图滤波
     * @note
     * @todo
     **/
    bool Matching::ConfigBoxFilter(const YAML::Node &config_node)
    {
        box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);
    }

    /**
     * @brief 重定位算法--加载全局地图
     * @note
     * @todo
     **/
    bool Matching::LoadGlobalMap()
    {
        /*加载原始地图*/
        pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
        LOG(INFO) << " [global map origin size]" << std::endl
                  << global_map_ptr_->points.size() << std::endl;
        /*降采样原始地图*/
        local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
        LOG(INFO) << "[global map filter size]" << std::endl
                  << global_map_ptr_->points.size() << std::endl;
        /*刷新标志位*/
        has_new_global_map_ = true;

        return true;
    }

    /**
     * @brief 重定位算法--重置局部地图
     * @note
     * @todo
     **/
    bool Matching::ResetLocalMap(float x, float y, float z)
    {
        /*设置起点 ps:先设起点*/
        box_filter_ptr_->SetOrigin(std::vector<float>{x, y, z});

        /*切割出子图*/
        box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

        /*设置输入点云 向target匹配*/
        registration_ptr_->SetInputTarget(local_map_ptr_);
        /*刷新标志位*/
        has_new_local_map_ = true;
        /*子图切割结果*/
        std::vector<float> edge = box_filter_ptr_->GetEdge();
        LOG(INFO) << " [new local map]" << std::endl
                  << edge.at(0) << " "
                  << edge.at(1) << " "
                  << edge.at(2) << " "
                  << edge.at(3) << " "
                  << edge.at(4) << " "
                  << edge.at(5) << std::endl;
        return true;
    }

    /**
     * @brief  判断是否经过gnss初始化
     * @note
     * @todo
     **/
    bool Matching::HasInited()
    {
        return has_inited_;
    }

    /**
     * @brief 判断是否有新的全局地图
     * @note
     * @todo
     **/
    bool Matching::HasNewGlobalMap()
    {
        return has_new_global_map_;
    }

    /**
     * @brief 判断是否有新的局部地图
     * @note
     * @todo
     **/
    bool Matching::HasNewLocalMap()
    {
        return has_new_local_map_;
    }

    /**
     * @brief 得到全局地图
     * @note
     * @todo
     **/
    void Matching::GetGlobalMap(CloudData::CLOUD_PTR &global_map)
    {
        global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
        has_new_global_map_ = false;
    }

    /**
     * @brief 得到局部地图
     * @note
     * @todo
     **/
    void Matching::GetLocalMap(CloudData::CLOUD_PTR &local_map)
    {
        local_map=local_map_ptr_;
        has_new_local_map_=false;
    }

    /**
     * @brief 得到当前扫描
     * @note
     * @todo
     **/
    void Matching::GetCurrentScan(CloudData::CLOUD_PTR &current_scan)
    {
        current_scan=current_scan_ptr_;
    }

        /**
     * @brief 初始位姿(目前由GNSS指定)
     * @note
     * @todo
     **/
    bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose)
    {
            init_pose_=init_pose;
            static int gnss_cnt=0;
            if(gnss_cnt==0)
            {
                ResetLocalMap(init_pose_(0,3),init_pose_(1,3),init_pose_(2,3));
            }
            else if(gnss_cnt>3)//等三帧稳定
            {
                has_inited_=true;
            }
            gnss_cnt++;
            return true;
    }

}; //  namespace multisensor_localization