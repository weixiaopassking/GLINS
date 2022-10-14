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
            /**
         * @brief 构造函数中开始计时
         * @note
         * @todo
         **/
        TimeRecorder()
        {
            start = std::chrono::system_clock::now();
        }

        /**
         * @brief 计时器结果输出
         * @note
         * @todo
         **/
        double Result()
        {
            end = std::chrono::system_clock::now();
            double res = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            start = std::chrono::system_clock::now();
            return res; //微妙级别
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
    };

} // namesapce multisensor_localization

#endif