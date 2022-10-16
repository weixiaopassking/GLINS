/*
 * @Description: 文件管理器
 * @Function: 增删改查文件
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

#ifndef  FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

//c++
#include <string>
#include <iostream>

namespace multisensor_localization
{

    class FileManager
    {
    public:
        static bool CreateDirectory(std::string directory_path);
        static bool CreateFile(std::ofstream &ofs, std::string file_path);
        static bool WriteFile(const std::ofstream &ofs, std::string file_path);
    };

}//namespace multisensor_localization

#endif