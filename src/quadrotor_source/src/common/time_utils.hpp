#ifndef _TIME_UTILS_HPP
#define _TIME_UTILS_HPP

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <numeric>

template <typename Func> void RuntimeRecord(const Func &func, const std::string &func_name, const int exec_cnt = 100)
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

#endif