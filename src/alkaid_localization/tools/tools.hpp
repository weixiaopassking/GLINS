/*
 * @Description: tools for debug
 * @Function: assert、time measure
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

#ifndef _TOOLS_HPP
#define _TOOLS_HPP

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include <math.h>

namespace tools_ns
{
#define StatusOutput true
#define VariableOutput true

/**
 * @brief    status output for debug
 * @param status_msg
 * @param function __FUNCTION__
 * @note
 **/
template <typename T> void StatusAssert(const T &status_msg, const char *function)
{
#if (StatusOutput == true)
    const std::string Green = "\033[0;32m";
    const std::string Reset = "\033[0m";
    const std::string header = "[" + static_cast<std::string>(function) + "]$ ";
    std::cout << Green << std::setw(10) << std::left << header << Reset << status_msg << std::endl;
#endif
}

/**
 * @brief     variables output for debug
 * @param Tn... multi params
 * @note
 **/
template <typename... Tn> void VariableAssert(Tn... tn)
{

    const std::string Purple = "\033[0;35m";
    const std::string Reset = "\033[0m";
#if (VariableOutput == true)

    const int len_f = sizeof...(tn);
    int cnt = 0;
    auto f = [&](auto it) {
        cnt == 0 ? std::cout << Purple << std::setw(10) << std::left << it << " " << Reset
                 : std::cout << it << std::endl;
        cnt ^= 1;
    };

    (..., f(tn)); // supported in  c++17

#endif
}

/**
 * @brief     variables output for debug
 * @param Tn... multi params __FILE__, __FUNCTION__, __LINE__
 * @note
 **/
template <typename T>
void ErrorAssert(const T &error_msg, const char *file, const char *function, const unsigned long line_num)
{
    const std::string Red = "\033[0;31m";
    const std::string Reset = "\033[0m";
    const std::string header = "[" + static_cast<std::string>(function) + "]$ ";
    const std::string position = static_cast<std::string>(file) + ":" + std::to_string(line_num);
    std::cout << Red << std::setw(10) << std::left << header << Reset << error_msg << std::endl;
    std::cout << "postion is:" << position << std::endl;

    exit(EXIT_FAILURE);
}

/**
 * @brief   record time cost
 * @param func  lambda expression
 * @note
 **/
template <typename Func> void TimeCost(const Func &func, const std::string &func_name, int exec_cnt = 1)
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
    double time_cost_variance = 0;
    std::for_each(time_cost_vec.begin(), time_cost_vec.end(),
                  [&time_cost_average, &time_cost_variance](const double time) {
                      time_cost_variance += std::pow(time_cost_average - time, 2);
                  });
    double time_cost_standard = std::sqrt(time_cost_variance / time_cost_vec.size());

    std::cout << std::setw(20) << std::left << func_name << std::endl
              << "time cost average: " << time_cost_average << " ms" << std::endl
              << "time cost 99 position: " << time_cost_99 << "ms" << std::endl
              << "time cost standard: " << time_cost_standard << "ms" << std::endl;
}

} // namespace tools_ns
#endif //_TOOLS_HPP