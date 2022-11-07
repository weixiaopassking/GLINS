/*
 * @Description: 程序计时器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef TIME_RECORDER_HPP_
#define TIME_RECORDER_HPP_

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace multisensor_localization
{

    class TimeRecorder
    {
    public:
        TimeRecorder();
        double Result();

    private:
        std::chrono::time_point<std::chrono::system_clock> start_, end_;
    };

}//namespace multisensor_localization

#endif