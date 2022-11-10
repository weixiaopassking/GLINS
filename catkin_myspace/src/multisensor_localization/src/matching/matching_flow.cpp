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
        // global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
        // local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
        // laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");
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
        /*判断地图是否成功加载*/

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
    }
    /**
     * @brief 重定位任务管理器件--检查数据
     * @note
     * @todo
     **/
    bool MatchingFlow::HasData()
    {
    }
    /**
     * @brief 重定位任务管理器件--提取有效数据
     * @note
     * @todo
     **/
    bool MatchingFlow::ValidData()
    {
    }
    /**
     * @brief 重定位任务管理器件--匹配更新
     * @note
     * @todo
     **/
    bool MatchingFlow::UpdateMatching()
    {
    }
    /**
     * @brief 重定位任务管理器件--数据发布
     * @note
     * @todo
     **/
    bool MatchingFlow::PublishData()
    {
    }

} // namespace multisensor_localization
