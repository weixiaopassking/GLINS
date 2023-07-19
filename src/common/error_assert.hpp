#ifndef _ERROR_ASSERT_HPP
#define _ERROR_ASSERT_HPP


#include <algorithm>
#include <iostream>
#include <string>

// Reset
#define Color_Off "\033[0m"
// Regular Colors
#define Black '\033[0;30m'
#define Red "\033[0;31m"
#define Green '\033[0;32m'
#define Yellow '\033[0;33m'
#define Blue '\033[0;34m'
#define Purple '\033[0;35m'
#define Cyan '\033[0;36m'
#define White '\033[0;37m'
// Background
#define On_Red "\033[41m"
// Bold High Intensity
#define BIRed "\033[1;91m"

enum ErrorCode
{
    empty_path,
    invalid_pcd,
     unknown
};

static std::unordered_map<enum ErrorCode, std::string> ErrorMap{
    {ErrorCode::empty_path, "路径空"},
    {ErrorCode::invalid_pcd, "无效点云"},
    {ErrorCode::unknown, "未知错误"},
};

void ErrorAssert(enum ErrorCode, const char *file = "null", const char *function = "null",
                 const unsigned long line = 0);

#endif  //_ERROR_ASSERT_HPP