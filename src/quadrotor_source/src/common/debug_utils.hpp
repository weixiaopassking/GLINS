/**
*****************************************************************************
*  Copyright (C), 2023-2026,wengang.niu
*  @file    debug_utils.hpp
*  @brief  debug工具
*  @author  wengang.niu
*  @date    2023/7/27
*  @version v0.2
*  @ref 
*  @note
****************************************************************************
*/

#ifndef _DEBUG_UTILS_HPP
#define _DEBUG_UTILS_HPP

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <numeric>

namespace common_ns
{
/**
 * @brief 运行时间统计
 * @param func 待函数lambda形式
 * @param  func_name 待测函数名
 * @param exec_cnt 运行次数 默认1次
 * @todo
 * @note
 **/
template <typename Func> void RuntimeRecord(const Func &func, const std::string &func_name, const int exec_cnt = 1)
{
    std::vector<double> time_cost_vec(exec_cnt);
    for (int times = 0; times < exec_cnt; times++)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        func();
        auto end_time = std::chrono::high_resolution_clock::now();

        time_cost_vec[times] = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    }

    sort(time_cost_vec.begin(), time_cost_vec.end(), [](const double t1, const double t2) -> bool { return t1 < t2; });

    double time_cost_average = std::accumulate(time_cost_vec.begin(), time_cost_vec.end(), 0) / exec_cnt;
    double time_cost_99 = time_cost_vec[static_cast<int>(0.99 * exec_cnt - 1)];
    double time_cost_var = 0;
    std::for_each(time_cost_vec.begin(), time_cost_vec.end(), [&time_cost_average, &time_cost_var](const double time) {
        time_cost_var += std::pow(time_cost_average - time, 2);
    });
    time_cost_var /= time_cost_vec.size();
    double time_cost_dev = std::sqrt(time_cost_var);

    std::cout << std::setw(20) << std::left << func_name << "平均耗时为: " << time_cost_average << " ms" << std::endl;
    //   << "99位耗时为: " << time_cost_99 << "ms" << std::endl
    //   << "耗时标准差为: " << time_cost_dev << "ms" << std::endl;
}

/**
 * @brief 状态量打印
 * @param tn 不定参数
 * @todo
 * @note
 **/
template <typename... Tn> void VariableInfo(Tn... tn)
{
    const int len_f = sizeof...(tn);

    int cnt = 0;
    auto f = [&](auto it) {
        cnt == 0 ? std::cout << std::setw(10) << std::left << it << " "
                 : std::cout  << it << std::endl;
                 cnt ^= 1;
    };

    (..., f(tn)); // 一元左折 c++17折叠表达式
    if (len_f%2!=0)
    {
        std::cout << std::endl;
    }
      
}

/**
 * @brief 错误断言
 * @param error_msg 错误信息or 错误代号
 * @param file 文件位置
 * @param function 函数位置
 * @param function 行号
 * @todo
 * @note
 **/
template <typename T> void ErrorAssert(T &error_msg, const char *file, const char *function, const unsigned long line)
{
    if (typeid(error_msg) == typeid(std::string))
    {
        std::cout << "错误信息:" << error_msg << std::endl;
    }
    else // 数字代码
    {
        std::cout << "错误代码:" << error_msg << std::endl;
    }

    std::cout << "错误位置:" << std::endl
              << "--文件:" << file << std::endl
              << "--函数:" << function << std::endl
              << "--行号:" << line << std::endl;
    exit(EXIT_FAILURE);
}
} // namespace common_ns
#endif

// /*彩色终端*/
// // Reset
// #define Color_Off "\033[0m"
// // Regular Colors
// #define Black "\033[0;30m"
// #define Red "\033[0;31m"
// #define Green "\033[0;32m"
// #define Yellow "\033[0;33m"
// #define Blue "\033[0;34m"
// #define Purple "\033[0;35m"
// #define Cyan "\033[0;36m"
// #define White "\033[0;37m"
// // Background
// #define On_Red "\033[41m"
// // Bold High Intensity
// #define BIRed "\033[1;91m"

// https: // stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux