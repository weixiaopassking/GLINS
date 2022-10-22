/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../include/mapping/loop_closing/loop_closing_flow.hpp"
// sensor_data
#include "../../../include/sensor_data/loop_pose.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"

namespace multisensor_localization
{

    /**
     * @brief 回环检测任务管理器--初始化
     * @note
     * @todo
     **/
    LoopClosinigFlow::LoopClosinigFlow(ros::NodeHandle &nh)
    {
        // subscriber
        key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 1e5);
        key_gnss_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_gnss", 1e5);
        // publisher
        loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(nh, "/loop_pose", "/map", 1e2);
        // loop closing
        loop_closing_ptr_ = std::make_shared<LoopClosing>();

        ColorTerminal::ColorFlowInfo("Flow配置完成");
    }

    /**
     * @brief 回环检测任务管理器--执行
     * @note
     * @todo
     **/
    bool LoopClosinigFlow::Run()
    {
        if (!ReadData())
            return false;

        while (HasData())
        {
            if (!ValidData())
                continue;

            loop_closing_ptr_->Update(current_key_frame_, current_key_gnss_);

            PublishData();
        }

        return true;
    }

    /**
     * @brief 回环检测任务管理器--读取传感器数据
     * @note
     * @todo
     **/
    bool LoopClosinigFlow::ReadData()
    {
        key_frame_sub_ptr_->ParseData(key_frame_buff_);
        key_gnss_sub_ptr_->ParseData(key_gnss_buff_);

        return true;
    }

    /**
     * @brief 回环检测任务管理器--提取有效数据
     * @note
     * @todo
     **/
    bool LoopClosinigFlow::ValidData()
    {
        current_key_frame_ = key_frame_buff_.front();
        current_key_gnss_ = key_gnss_buff_.front();

        double diff_gnss_time = current_key_frame_.time_stamp_ - current_key_gnss_.time_stamp_;

        if (diff_gnss_time < -0.05)
        {
            key_frame_buff_.pop_front();
            return false;
        }

        if (diff_gnss_time > 0.05)
        {
            key_gnss_buff_.pop_front();
            return false;
        }

        key_frame_buff_.pop_front();
        key_gnss_buff_.pop_front();

        return true;
    }

    /**
     * @brief 回环检测任务管理器--检测是否有数据
     * @note
     * @todo
     **/
    bool LoopClosinigFlow::HasData()
    {
        if (key_frame_buff_.size() == 0)
            return false;
        if (key_gnss_buff_.size() == 0)
            return false;

        return true;
    }

    /**
     * @brief 回环检测任务管理器--发布数据
     * @note
     * @todo
     **/
    bool LoopClosinigFlow::PublishData()
    {
        return true;
    }

} // namespace multisensor_localization