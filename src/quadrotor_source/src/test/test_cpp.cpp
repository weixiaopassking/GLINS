/*注:
quick_test
仅仅用于C++/Python  及第三方库使用快速验证
*/

#include "../common/debug_utils.hpp"
#include "vector"
#include <execution>
#include <gtest/gtest.h> //单元测试
#include <iostream>
#include <random>

#define SEQ std::execution::seq
#define PAR std::execution::par
#define PAR_UNSEQ std::execution::par_unseq

/*
sort并行并发比较
性能:PAR>PAR_UNSEQ>None>SEQ
*/
TEST(example, instance1)
{
    std::random_device rd;  // 随机数发生器
    std::mt19937 gen(rd()); // 随机数引擎
    std::uniform_real_distribution<> distrib(0, 100);
    std::array<std::vector<double>, 4> num_table;

    for (int i = 0; i < 1e8; i++)
    {
        num_table[0].push_back(distrib(gen));
        num_table[1].push_back(distrib(gen));
        num_table[2].push_back(distrib(gen));
        num_table[3].push_back(distrib(gen));
    }
    common_ns::RuntimeRecord([&num_table]() { std::sort(num_table[0].begin(), num_table[0].end()); }, "sort-none");
    common_ns::RuntimeRecord([&num_table]() { std::sort(SEQ, num_table[1].begin(), num_table[1].end()); }, "sort-seq");
    common_ns::RuntimeRecord([&num_table]() { std::sort(PAR, num_table[2].begin(), num_table[2].end()); }, "sort-par");
    common_ns::RuntimeRecord([&num_table]() { std::sort(PAR_UNSEQ, num_table[3].begin(), num_table[3].end()); },
                             "sort-par_unseq");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}