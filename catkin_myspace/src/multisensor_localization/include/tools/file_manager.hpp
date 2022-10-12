#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

#include <string>
#include <iostream>
namespace multisensor_localization
{

    class FileManager
    {
    public:
    static bool CreateDirectory(std::string directory_path);
    static bool CreateFile(std::string file_path);
    static bool WriteFile(const std::ofstream &ofs,std::string file_path); 
    };

}

#endif