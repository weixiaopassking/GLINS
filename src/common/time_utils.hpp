#ifndef _TIME_UTILS_HPP
#define _TIME_UTILS_HPP
#include <algorithm>//stl
#include <chrono>
#include <iostream>
#include <numeric>
#include <math.h>

template <typename Func> void RuntimeRecord(const Func& func, const std::string &func_name, const int exec_cnt=100)
{
    std::vector<double> time_cost_vec(exec_cnt);
    for (int times = 0; times < exec_cnt; times++)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        func();
        auto end_time = std::chrono::high_resolution_clock::now();

        time_cost_vec[times] = std::chrono::duration<double, std::ratio<1, 1000>>(end_time - start_time).count();
    }

    sort(time_cost_vec.begin(), time_cost_vec.end(),[](const double t1,const double t2)->bool{return t1<t2; });
    double time_cost_average = std::accumulate(time_cost_vec.begin(), time_cost_vec.end(), 0) / exec_cnt;
    double time_cost_99 = time_cost_vec[static_cast<int>(0.99 * exec_cnt - 1)];
    double time_cost_var =0;
    std::for_each(time_cost_vec.begin(), time_cost_vec.end(), [&time_cost_average, &time_cost_var](const double time) {
        time_cost_var +=std::pow(time_cost_average - time,2);
    });
    time_cost_var /= time_cost_vec.size();
    double time_cost_dev = std::sqrt(time_cost_var);
    std::cout << func_name << std::endl
              << "平均耗时为: " << time_cost_average << "ms" << std::endl
              << "99位耗时为: " << time_cost_99 << "ms" << std::endl
              << "耗时标差为: " << time_cost_dev << "ms" << std::endl;
}


#endif