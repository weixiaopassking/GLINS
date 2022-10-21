/*
 * @Description: 回环检测任务管理器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-21
 */

#ifndef LOOP_CLOSING_FLOW_HPP_
#define LOOP_CLOSING_FLOW_HPP_

//c++
#include <deque>
// ros
#include <ros/ros.h>
// sensor data
#include "../../../include/sensor_data/key_frame.hpp"
//subscriber

//publisher

namespace multisensor_localization
{
    class LoopClosinigFlow
    {
    public:
        LoopClosinigFlow(ros::NodeHandle &nh);
        bool Run();

    private:
        bool ReadData();


        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> gnss_frame_buff_;

    }; // class LoopClosinigFlow

    
} // namespace multisensor_localization

#endif
