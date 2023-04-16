/*
 * @Description: 程序计时器
 * @Function:
 * @Author: Robotic Gang (modified from Ren Qian)
 * @Version : v1.0
 * @Date: 2022-10-16
 */

// relevent
#include "../../include/tools/file_manager.hpp"
// boost
#include <boost/filesystem.hpp>

namespace multisensor_localization
{
    /**
     * @brief 创建文件夹
     * @note
     * @todo
     **/
    bool FileManager::CreateDirectory(std::string directory_path)
    {
        /*是否已存在该文件夹 存在则删除*/
        if (boost::filesystem::is_directory(directory_path))
        {
            boost::filesystem::remove_all(directory_path);
        }
        /*尝试创建新文件夹,抓取异常*/
        try
        {
            boost::filesystem::create_directory(directory_path);
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            return false;
        }
        /*检查slam_data文件夹是否创建成功*/
        if (boost::filesystem::is_directory(directory_path))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * @brief 创建文件
     * @note
     * @todo
     **/
    bool FileManager::CreateFile(std::ofstream &ofs, std::string file_path)
    {
        ofs.close();
        boost::filesystem::remove(file_path.c_str());

        ofs.open(file_path.c_str(), std::ios::app);
        if (!ofs)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 写入文件
     * @note
     * @todo 向末尾写入文件
     **/
    bool FileManager::WriteFile(const std::ofstream &ofs, std::string file_path)
    {
        return true;
    }

} // namespace multisensor_localization