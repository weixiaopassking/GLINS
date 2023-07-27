#ifndef _DEBUG_INFO_HPP
#define _DEBUG_INFO_HPP

#include <iostream>
#include <unordered_map>

/*彩色终端*/
// Reset
#define Color_Off "\033[0m"
// Regular Colors
#define Black "\033[0;30m"
#define Red "\033[0;31m"
#define Green "\033[0;32m"
#define Yellow "\033[0;33m"
#define Blue "\033[0;34m"
#define Purple "\033[0;35m"
#define Cyan "\033[0;36m"
#define White "\033[0;37m"
// Background
#define On_Red "\033[41m"
// Bold High Intensity
#define BIRed "\033[1;91m"

/*变量快捷打印*/
template <typename... Tn> void VariableInfo(Tn... tn)
{
    const int len_f = sizeof...(tn);

    int cnt = 0;
    auto f = [&](auto it) {
        cnt == 0 ? std::cout << Yellow << it << ":\t" << Color_Off
                 : std::cout << Cyan << it << " " << Color_Off << std::endl;
        cnt ^= 1;
    };

    (..., f(tn)); // 一元左折 c++17折叠表达式
}

/*错误断言*/
enum ErrorCode
{
    error_path, // 路径错误
    error_file, // 文件错误
    unknown     // 暂时未知
};

static std::unordered_map<enum ErrorCode, std::string> ErrorMap{
    {ErrorCode::error_path, "路径错误"},
    {ErrorCode::error_file, "内容错误"},
    {ErrorCode::unknown, "未知错误"},
};

void ErrorAssert(enum ErrorCode, const char *file = "null", const char *function = "null",
                 const unsigned long line = 0);

// example: ErrorAssert(ErrorCode::error_path, __FILE__, __FUNCTION__, __LINE__);

#endif // _DEBUG_INFO_HPP