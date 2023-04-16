/*
 * @Description: 速度数据存放
 * @Function: 线速度+角速度
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 * @Todo
 */
#include "../../include/sensor_data/velocity_data.hpp"

namespace multisensor_localization
{
    /**
     * @brief 时间同步
     * @note
     * @todo
     **/
    bool VelocityData::SyncData(std::deque<VelocityData> &unsynced_data_buff,
                                std::deque<VelocityData> &synced_data_buff, double sync_time)
    {

        while (unsynced_data_buff.size() >= 2)
        {
            /*异常1:sync_time<[0]<[1]*/
            if (unsynced_data_buff.at(0).time_stamp_ > sync_time)
                return false;
            /*异常2:[0]<[1]<sync_time*/
            if (unsynced_data_buff.at(1).time_stamp_ < sync_time)
            {
                unsynced_data_buff.pop_front();
                continue;
            }
            /*异常3:[0]<<sync_time<[1]*/
            if (sync_time - unsynced_data_buff.at(0).time_stamp_ > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            /*异常4:[0]<sync_time<<[2]*/
            if (unsynced_data_buff.at(1).time_stamp_ - sync_time > 0.2)
            {
                unsynced_data_buff.pop_front();
                break;
            }
            break;
        }

        if (unsynced_data_buff.size() < 2)
            return false;

        /*线性插值a(1-t)+bt系数计算*/
        VelocityData front_data = unsynced_data_buff.at(0);
        VelocityData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) / (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) / (back_data.time_stamp_ - front_data.time_stamp_);

        VelocityData synced_data;

        /*非同步量拷贝*/
        synced_data.time_stamp_ = sync_time;
        /*同步量拷贝*/
        synced_data.linear_velocity_.x = front_data.linear_velocity_.x * front_scale + back_data.linear_velocity_.x * back_scale;
        synced_data.linear_velocity_.y = front_data.linear_velocity_.y * front_scale + back_data.linear_velocity_.y * back_scale;
        synced_data.linear_velocity_.z = front_data.linear_velocity_.z * front_scale + back_data.linear_velocity_.z * back_scale;

        synced_data.angular_velocity_.x = front_data.angular_velocity_.x * front_scale + back_data.angular_velocity_.x * back_scale;
        synced_data.angular_velocity_.y = front_data.angular_velocity_.y * front_scale + back_data.angular_velocity_.y * back_scale;
        synced_data.angular_velocity_.z = front_data.angular_velocity_.z * front_scale + back_data.angular_velocity_.z * back_scale;

        synced_data_buff.push_back(synced_data);

        return true;
    }

    /**
     * @brief 臂杆速度传递
     * @note 计算gnss系测量出来的线/角速度传递到lidar系
     * 原理:连杆间的速度传递
     * 输入参数:imu_to_lidar
     * @todo
     **/
    void VelocityData::TransformCoordinate(Eigen::Matrix4f transform_matrix)
    {
        Eigen::Matrix4d matrix = transform_matrix.cast<double>();
        Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d w(angular_velocity_.x, angular_velocity_.y, angular_velocity_.z);
        Eigen::Vector3d v(linear_velocity_.x, linear_velocity_.y, linear_velocity_.z);
        /*{i+1}系统的角速度*/
        w = t_R * w;
        /*{i+1}系统的线速度*/
        v = t_R * v;
        Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
        Eigen::Vector3d delta_v;
        delta_v(0) = w(1) * r(2) - w(2) * r(1);
        delta_v(1) = w(2) * r(0) - w(0) * r(2);
        delta_v(2) = w(0) * r(1) - w(1) * r(0);
        v = v + delta_v;
        /*结果传入类内全局变量*/
        angular_velocity_.x = w(0);
        angular_velocity_.y = w(1);
        angular_velocity_.z = w(2);
        linear_velocity_.x = v(0);
        linear_velocity_.y = v(1);
        linear_velocity_.z = v(2);
    }

} // namespace multisensor_localization