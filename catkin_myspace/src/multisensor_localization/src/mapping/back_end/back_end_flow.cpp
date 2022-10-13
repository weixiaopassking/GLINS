/*
 * @Description:传感器数据预处理
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-03
 */

#include "../../../include/mapping/back_end/back_end_flow.hpp"
#include "../../../include/sensor_data/cloud_data.hpp"
#include "../../../include/subscriber/odometry_subscriber.hpp"
#include "../../../include/publisher/odometry_publisher.hpp"
#include "../../../include/publisher/key_frame_publisher.hpp"
#include "../../../include/publisher/key_frames_publisher.hpp"

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
        while (!HasData())
        {
            if (ValidData())
                continue;

            //更新后端
            UpdateBackEnd();
            //发布数据
        }

        return true;
    }
    /**
     * @brief 后端流任务管理 数据读取
     * @note
     * @todo
     **/
    bool BackEndFlow::ReadData()
    {
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        gnss_odom_sub_ptr_->ParseData(gnss_odom_data_buff_);
        laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
        return true;
    }

    /**
     * @brief 后端流任务管理 数据确认
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

        /*根据时间戳对齐刷新数据*/
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
    }

    /**
     * @brief  更新后端(核心)
     * @note
     * @todo
     **/
    bool BackEndFlow::UpdateBackEnd()
    {
        /*lidar 对齐到gnss坐标系再优化*/
        static bool odom_inited = false;
        static Eigen::Matrix4f gnss_to_lidar_matrix = Eigen::Matrix4f::Identity();
        if (!odom_inited)
        {
            odom_inited = true;
            gnss_to_lidar_matrix = current_gnss_pose_data_.pose_ * current_laser_odom_data_.pose_.inverse();
        }
        current_laser_odom_data_.pose_ = gnss_to_lidar_matrix * current_laser_odom_data_.pose_;
        return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
    }

    /**
     * @brief  保存轨迹数据
     * @note
     * @todo
     **/
    bool BackEndFlow::SaveTrajectory()
    {
    }

    /**
     * @brief  保存轨迹数据
     * @note
     * @todo
     **/



} // namespace multisensor_localization
