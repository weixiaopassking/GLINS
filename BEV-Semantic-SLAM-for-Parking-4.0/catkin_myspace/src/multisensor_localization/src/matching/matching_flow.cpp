/*
 * @Description: 重定位任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-11-08
 */

// relevent
#include "../../include/matching/matching_flow.hpp"
//算法层
#include "../../include/matching/matching.hpp"
// tools
#include "../../include/tools/color_terminal.hpp"
// pcl
#include <pcl/common/transforms.h>
// pub
#include "../../include/publisher/tf_broadcaster.hpp"

namespace multisensor_localization
{
    /**
     * @brief 重定位任务管理器件--初始化
     * @note
     * @todo
     **/
    MatchingFlow::MatchingFlow(ros::NodeHandle &nh)
    {
        /*订阅*/
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 1e5);
        /*发布*/
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
        laser_tf_pub_ptr_ = std::make_shared<TfBroadcaster>("/map", "/vehicle_link");
        /*匹配算法*/
        matching_ptr_ = std::make_shared<Matching>();
    }

    /**
     * @brief 重定位任务管理器件--逻辑执行
     * @note
     * @todo
     **/
    bool MatchingFlow::Run()
    {
        /*判断全局是否成功加载且被订阅*/
        if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers())
        {
            CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
            matching_ptr_->GetGlobalMap(global_map_ptr);
            global_map_pub_ptr_->Publish(global_map_ptr);
            ColorTerminal::ColorFlowInfo("全局地图已加载");
        }
        /*判断局部是否加载成功且并订阅*/
        if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        {
            CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
            matching_ptr_->GetLocalMap(local_map_ptr);
            local_map_pub_ptr_->Publish(local_map_ptr);
            ColorTerminal::ColorFlowInfo("局部地图已更新");
        }

        /*读取数据*/
        ReadData();
        /*校验数据*/
        while (HasData())
        {
            if (!ValidData())
                continue;
            /*匹配更新*/
            if (UpdateMatching())
            {

                PublishData();
            }
        }
    }

    /**
     * @brief 重定位任务管理器件--读取数据
     * @note
     * @todo
     **/
    bool MatchingFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        gnss_sub_ptr_->ParseData(gnss_data_buff_);
        return true;
    }
    /**
     * @brief 重定位任务管理器件--检查数据
     * @note
     * @todo
     **/
    bool MatchingFlow::HasData()
    {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (matching_ptr_->HasInited()) //初始化后不再需要gnss数据了
            return true;
        if (gnss_data_buff_.size() == 0)
            return false;
        return true;
    }
    /**
     * @brief 重定位任务管理器件--提取有效数据
     * @note
     * @todo
     **/
    bool MatchingFlow::ValidData()
    {
        current_cloud_data_ = cloud_data_buff_.front();

        /*初始化后也不再需要多传感器对齐*/
        if (matching_ptr_->HasInited())
        {
            cloud_data_buff_.pop_front();
            gnss_data_buff_.clear();
            return true;
        }
        current_gnss_data_ = gnss_data_buff_.front();

        /*时间对齐检验 以激光雷达为基准*/
        double diff_time = current_cloud_data_.time_stamp_ - current_gnss_data_.time_stamp_;
        if (diff_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (diff_time > 0.05)
        {
            gnss_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        return true;
    }
    /**
     * @brief 重定位任务管理器件--匹配更新
     * @note
     * @todo
     **/
    bool MatchingFlow::UpdateMatching()
    {
        /*以gnss信号初始化*/
        if (!matching_ptr_->HasInited())
        {
            matching_ptr_->SetInitPose(current_gnss_data_.pose_);
            ColorTerminal::ColorFlowInfo("初始位置已给定"); //仅仅一次
            return true;
        }
        /*地图匹配更新(核心)*/
        return matching_ptr_->Update(current_cloud_data_, laser_odometry_);
    }
    /**
     * @brief 重定位任务管理器件--数据发布
     * @note
     * @todo
     **/
    bool MatchingFlow::PublishData()
    {
        std::cout << "定位结果:" << std::endl;
        std::cout << laser_odometry_ << std::endl;
        /*激光定位tf*/
        laser_tf_pub_ptr_->SendTransform(laser_odometry_, current_cloud_data_.time_stamp_);
        /*激光里程计*/
        laser_odom_pub_ptr_->Publish(laser_odometry_);
        /*当前激光扫描*/
        // pcl::PointCloudColorHandlerCustom();
        CloudData::CLOUD_PTR current_scan_display(new CloudData::CLOUD());
        matching_ptr_->GetCurrentScan(current_scan_display);
        current_scan_pub_ptr_->Publish(current_scan_display);

        return true;
    }

} // namespace multisensor_localization
