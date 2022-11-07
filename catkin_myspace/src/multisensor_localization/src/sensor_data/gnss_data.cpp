/*
 * @Description: gnss数据封装
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-14
 */

// relevant
#include "../../include/sensor_data/gnss_data.hpp"
// glog
#include <glog/logging.h>
// c++
#include "bits/stdc++.h"

namespace multisensor_localization
{
    /*类内静态变量 类内申明类外初始化*/
    bool GnssData::origin_position_inited_ = false;
    GeographicLib::LocalCartesian GnssData::geo_converter_;

    /**
     * @brief 初始化东北天坐标系原点
     * @note
     * @todo
     **/
    void GnssData::InitOriginPosition()
    {
        geo_converter_.Reset(latitude_, longitude_, altitude_);
        origin_position_inited_ = true;
    }

    /**
     * @brief 更新东北天三轴位移
     * @note
     * @todo
     **/
    void GnssData::UpdateXYZ()
    {
        if (!origin_position_inited_)
        {
            LOG(ERROR) << "[东北天坐标系未正常初始化]" << std::endl;
        }
        /*取出新位移*/
        geo_converter_.Forward(latitude_, longitude_, altitude_,
                               local_E_, local_N_, local_U_);
    }

    /**
     * @brief gnss时间同步
     * @note
     * @todo
     **/
    bool GnssData::SyncData(std::deque<GnssData> &unsynced_data_buff,
                            std::deque<GnssData> &synced_data_buff, double sync_time)
    {

        while (unsynced_data_buff.size() >= 2)
        {
            std::cout<<"just a test"<<std::endl;
            std::cout<<std::fixed << unsynced_data_buff.at(0).time_stamp_ << std::endl;
            std::cout << std::fixed<<unsynced_data_buff.at(1).time_stamp_ << std::endl;
            std::cout <<std::fixed<< sync_time << std::endl;

            /*异常1:sync_time<[0]<[1] #无法对齐,退出*/
            if (unsynced_data_buff.at(0).time_stamp_ > sync_time)
                return false;
            /*异常2:[0]<[1]<sync_time  #丢掉[0],继续下一轮循环尝试转用[1][2]*/
            if (unsynced_data_buff.at(1).time_stamp_ < sync_time)
            {
                unsynced_data_buff.pop_front();
                continue;
            }
            // /*异常3:[0]<<sync_time<[1] #[0]与sync_time可能丢帧,弃用[0]，退出  */
            // if (sync_time - unsynced_data_buff.at(0).time_stamp_ > 0.2)
            // {
            //     unsynced_data_buff.pop_front();
            //     break;
            // }
            // /*异常4:[0]<sync_time<<[2]  #sync_time与[1]可能丢帧,弃用[0],，退出*/
            // if (unsynced_data_buff.at(1).time_stamp_ - sync_time > 0.2)
            // {
            //     unsynced_data_buff.pop_front();
            //     break;
            // }
            break;
        }

        /*经上述处理后，待同步数据不足对，无法同步*/
        if (unsynced_data_buff.size() < 2)
            return false;

        /*线性插值a(1-t)+bt 系数计算*/
        GnssData front_data = unsynced_data_buff.at(0);
        GnssData back_data = unsynced_data_buff.at(1);
        double front_scale = (back_data.time_stamp_ - sync_time) / (back_data.time_stamp_ - front_data.time_stamp_);
        double back_scale = (sync_time - front_data.time_stamp_) / (back_data.time_stamp_ - front_data.time_stamp_);

        GnssData synced_data;

        /*非同步量拷贝*/
        synced_data.time_stamp_ = sync_time;
        synced_data.status_ = back_data.status_;
        /*同步量拷贝*/
        synced_data.longitude_ = front_data.longitude_ * front_scale + back_data.longitude_ * back_scale;
        synced_data.latitude_ = front_data.latitude_ * front_scale + back_data.latitude_ * back_scale;
        synced_data.altitude_ = front_data.altitude_ * front_scale + back_data.altitude_ * back_scale;

        synced_data.local_E_ = front_data.local_E_ * front_scale + back_data.local_E_ * back_scale;
        synced_data.local_N_ = front_data.local_N_ * front_scale + back_data.local_N_ * back_scale;
        synced_data.local_U_ = front_data.local_U_ * front_scale + back_data.local_U_ * back_scale;

        /*压入队列引用方式向外传参*/
        synced_data_buff.push_back(synced_data);

        return true;
    }

} // multisensor_localization