/*注:
quick_test
仅仅用于C++/Python  及第三方库使用快速验证
*/
#include <chrono>
#include <execution>
#include <gtest/gtest.h> //单元测试
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <utility>
#include <vector>
using namespace std;

#define SEQ std::execution::seq
#define PAR std::execution::par
#define PAR_UNSEQ std::execution::par_unseq

TEST(example, instance1)
{
    std::random_device rd;  // 随机数发生器
    std::mt19937 gen(rd()); // 随机数引擎
    std::uniform_real_distribution<> distrib(0, 100);
    // uniform_real_distribution
    vector<double> num_vec;

    for (int i = 0; i < 1e5; i++)
    {
        num_vec.push_back(distrib(gen));
    }

    auto eval = [](auto fun) {
        const auto t1 = std::chrono::high_resolution_clock::now();
        const auto name = fun();
        const auto t2 = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<double, std::milli> ms = t2 - t1;
        std::cout << std::setw(28) << std::left << name << "\t time: " << ms.count() << " ms\n";
    };

    {

        eval([&num_vec] {
            std::sort(num_vec.begin(), num_vec.end());
            return "std::sort ";
        });

        eval([&num_vec] {
            std::sort(SEQ, num_vec.begin(), num_vec.end());
            return "std::sort  SEQ";
        });

        eval([&num_vec] {
            std::sort(PAR_UNSEQ, num_vec.begin(), num_vec.end());
            return "std::sort  PAR_UNSEQ";
        });

        eval([&num_vec] {
            std::sort(PAR, num_vec.begin(), num_vec.end());
            return "std::sort  PAR";
        });
    }
}


    int main(int argc, char **argv)
    {
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }