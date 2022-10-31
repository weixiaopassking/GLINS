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
        // subscriber
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 1e5);
        transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 1e5);
        optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 1e5);
        // publisher
        optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/optimized_odom", "/map", "/lidar", 100);
        current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
        global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
        local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    }

} // namespace  multisensor_localization