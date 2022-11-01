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
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 1e5);
        transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 1e5);
        optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 1e5);
        /*publisher*/
        optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "/map", "/lidar", 100);
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
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
        while (HasData())
        {
            if (ValidData())
            {
                // viewer_ptr_->UpdateNewKeyFrame();
                PublishLocalData();
            }
        }

        if (optimized_key_frames_buff_.size() > 0)
        {
            // viewer_ptr_->UpdateNewKeyFrame();
            PublishGlobalData();
        }
        return true;
    }

    /**
     * @brief 读取数据
     * @note
     * @todo
     **/
    bool ViewerFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
        key_frame_sub_ptr_->ParseData(key_frame_buff_);
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
        if (cloud_data_buff_.size() == 0)
            return false;
        if (transformed_odom_buff_.size() == 0)
            return false;

        return true;
    }

    /**
     * @brief提取有效数据
     * @note
     * @todo
     **/
    bool ViewerFlow::ValidData()
    {
        current_cloud_data_ = cloud_data_buff_.front();
        current_transformed_odom_ = transformed_odom_buff_.front();

        double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;

        if (diff_odom_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (diff_odom_time > 0.05)
        {
            transformed_odom_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        transformed_odom_buff_.pop_front();

        return true;
    }

} // namespace  multisensor_localization