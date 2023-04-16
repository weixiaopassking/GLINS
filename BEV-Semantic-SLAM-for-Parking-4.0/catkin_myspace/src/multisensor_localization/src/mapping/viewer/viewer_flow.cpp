/*
 * @Description:可视化任务管理器
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

// relevent
#include "../../../include/mapping/viewer/viewer_flow.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"

namespace multisensor_localization
{
    /**
     * @brief 可视化流程初始化
     * @note
     * @todo
     **/
    ViewerFlow::ViewerFlow(ros::NodeHandle &nh)
    {
        /*subscriber*/
        // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        // key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 1e5);
        // transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 1e5);
        optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 1e5);
        /*publisher*/
        // optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "/map", "/lidar", 100);
        // current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
        // global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
        // local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
        /*view*/
        viewer_ptr_ = std::make_shared<Viewer>();
        /*终端提示*/
        ColorTerminal::ColorFlowInfo("可视化参数配置完成");
    }

    /**
     * @brief 可视化流程 运行
     * @note
     * @todo
     **/
    bool ViewerFlow::Run()
    {

        if (!ReadData())
        {
            return false;
        }
        // while (HasData())
        // {
        //     if (ValidData())
        //     {
        //     }
        // }

        return true;
    }

    /**
     * @brief 读取数据
     * @note
     * @todo
     **/
    bool ViewerFlow::ReadData()
    {
        optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);
        return true;
    }

    /**
     * @brief 检查数据
     * @note
     * @todo
     **/
    bool ViewerFlow::HasData()
    {
        return true;
    }

    /**
     * @brief提取有效数据
     * @note
     * @todo
     **/
    bool ViewerFlow::ValidData()
    {
        return true;
    }

    /**
     * @brief 发布全局数据
     * @note
     * @todo
     **/

    bool ViewerFlow::SaveMap()
    {
        viewer_ptr_->optimized_key_frames_=this->optimized_key_frames_;
        return viewer_ptr_->SaveMap();
    }

} // namespace  multisensor_localization