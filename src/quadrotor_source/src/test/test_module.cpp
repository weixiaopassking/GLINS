#include "../common/debug_utils.hpp"
#include <gtest/gtest.h> //单元测试

TEST(example, instance1)
{

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}