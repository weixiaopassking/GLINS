/*
 * @Description: 后端任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-19
 */

// relevent
#include "../../../include/mapping/back_end/back_end_flow.hpp"
// sensor_data
#include "../../../include/sensor_data/cloud_data.hpp"
// subscriber
#include "../../../include/subscriber/odometry_subscriber.hpp"
// publisher
#include "../../../include/publisher/odometry_publisher.hpp"
#include "../../../include/publisher/key_frame_publisher.hpp"
#include "../../../include/publisher/key_frames_publisher.hpp"
// tools
#include "../../../include/tools/color_terminal.hpp"

namespace multisensor_localization
{
    /**
     * @brief 后端任务管理器初始化
     * @note 话题订阅、话题发布、后端算法初始化
     * @todo 完结
     **/
    BackEndFlow::BackEndFlow(ros::NodeHandle &nh)
    {
        /*话题订阅*/
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1e5);
        gnss_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 1e5);
        laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odom", 1e5);

        /*话题发布*/
        transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);
        key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
        key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "/map", 100);

        /*后端优化*/
        back_end_ptr_ = std::make_shared<BackEnd>();
        ColorTerminal::ColorFlowInfo("Flow配置完成");
    }

    /**
     * @brief 后端流程控制初始化
     * @note 订阅点云信息 发布激光里程计
     * @todo
     **/
    bool BackEndFlow::Run()
    {
        if (!ReadData())
            return false;
        while (HasData())
        {
            if (!ValidData())
                continue;

            /*更新后端*/
            UpdateBackEnd();
            /*发布数据*/
            PublishData();
        }

        return true;
    }
    /**
     * @brief 后端流任务管理--数据读取
     * @note
     * @todo
     **/
    bool BackEndFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        gnss_odom_sub_ptr_->ParseData(gnss_odom_data_buff_);
        laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
        // loop_
        return true;
    }

    /**
     * @brief 后端流任务管理-- 数据确认
     * @note
     * @todo
     **/
    bool BackEndFlow::HasData()
    {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (gnss_odom_data_buff_.size() == 0)
            return false;
        if (laser_odom_data_buff_.size() == 0)
            return false;

        return true;
    }
    /**
     * @brief 后端流任务管理 有效数据提取
     * @note 计算下时间戳对齐程序,刷新数据
     * @todo
     **/
    bool BackEndFlow::ValidData()
    {
        /*提取出当前数据*/
        current_cloud_data_ = cloud_data_buff_.front();
        current_gnss_pose_data_ = gnss_odom_data_buff_.front();
        current_laser_odom_data_ = laser_odom_data_buff_.front();
        /*计算时间差*/
        double diff_gnss_time = current_cloud_data_.time_stamp_ - current_gnss_pose_data_.time_stamp_;
        double diff_laser_time = current_cloud_data_.time_stamp_ - current_laser_odom_data_.time_stamp_;

        /*根据时间戳对齐刷新数据 50ms 10hz*/
        if (diff_gnss_time < -0.05 || diff_laser_time < -0.05)
        {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (diff_gnss_time > 0.05)
        {
            gnss_odom_data_buff_.pop_front();
            return false;
        }

        if (diff_laser_time > 0.05)
        {
            laser_odom_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        gnss_odom_data_buff_.pop_front();
        laser_odom_data_buff_.pop_front();

        return true;
    }

    /**
     * @brief  后端优化(核心)
     * @note
     * @todo
     **/
    bool BackEndFlow::UpdateBackEnd()
    {
        /*计算gnss到lidar的转换 这里需要还思考思考*/
        static bool odom_inited = false;
        static Eigen::Matrix4f lidar_to_gnss_matrix = Eigen::Matrix4f::Identity();

        if (!odom_inited)
        {
            odom_inited = true;
            lidar_to_gnss_matrix = current_gnss_pose_data_.pose_ * current_laser_odom_data_.pose_.inverse();
        }
        /*lidar转到gnss下 对齐才能把一起优化*/
        current_laser_odom_data_.pose_ = lidar_to_gnss_matrix * current_laser_odom_data_.pose_;

        return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
    }

    /**
     * @brief  数据发布
     * @note
     * @todo
     **/
    bool BackEndFlow::PublishData()
    {
        /*发布优化前的位姿*/
        transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose_);

        /*以方向簇的方式发布新关键 */
        if (back_end_ptr_->HasNewKeyFrame())
        {
            KeyFrame key_frame;
            back_end_ptr_->GetCurrentKeyFrame(key_frame);
            key_frame_pub_ptr_->Publish(key_frame);
        }

        /*以路径形式发布优化后的关键帧序列*/
        if (back_end_ptr_->HasNewOptimized())
        {
            std::deque<KeyFrame> optimized_key_frames;
            back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
            key_frames_pub_ptr_->Publish(optimized_key_frames);
        }
        return true;
    }

    /**
     * @brief  flow层强制优化
     * @note 需要从接口中去算法层获取信息
     * @todo
     **/
    bool BackEndFlow::ForceOptimize()
    {
        back_end_ptr_->ForceOptimize();
        if (back_end_ptr_->HasNewKeyFrame())
        {
            std::deque<KeyFrame> optimized_key_frames;
            back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
            key_frames_pub_ptr_->Publish(optimized_key_frames);
        }
        return true;
    }

} // namespace multisensor_localization
