/*
 * @Description: 程序计时器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

//relevent
#include "../../include/tools/time_recorder.hpp"

namespace multisensor_localization
{
    /**
     * @brief 构造函数中开始计时
     * @note 毫秒级
     * @todo
     **/
    TimeRecorder::TimeRecorder()
    {
        start_ = std::chrono::system_clock::now();
    }

       /**
     * @brief 计时结束
     * @note 毫秒级
     * @todo
     **/
    double TimeRecorder::Result()
    {
        end_ = std::chrono::system_clock::now();
        double res = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start_).count();
        start_ = std::chrono::system_clock::now();
        return res; //微妙级别
    }

} //  namesapce multisensor_localization
