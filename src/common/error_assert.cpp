#include "error_assert.hpp"


void ErrorAssert(enum ErrorCode number, const char *file, const char *function, const unsigned long line)
{
    std::cout << BIRed << "错误信息:" << ErrorMap[number] << Color_Off << std::endl;
    std::cout << BIRed << "错误位置:" << std::endl
              << "--文件:" << file << std::endl
              << "--函数:" << function << std::endl
              << "--行号:" << line << Color_Off << std::endl;
    exit(EXIT_FAILURE);
}
// https: // stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux