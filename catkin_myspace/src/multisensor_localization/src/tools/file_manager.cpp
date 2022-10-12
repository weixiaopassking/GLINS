#include "../../include/tools/file_manager.hpp"
#include <boost/filesystem.hpp>
#include <glog/logging.h>

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
            // LOG(ERROR)  << "[文件夹路径非法]" << std::endl;
            return false;
        }
        /*检查slam_data文件夹是否创建成功*/
        if (boost::filesystem::is_directory(directory_path))
        {
            //  LOG(INFO) << "[文件夹创建成功]" << std::endl;
            return true;
        }
        else
        {
            LOG(ERROR) << "[数据文件夹创建失败]" << std::endl;
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
        ofs.open(file_path.c_str(),std::ios::app);
        if(!ofs)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 写入文件
     * @note
     * @todo
     **/
    bool FileManager::WriteFile(const std::ofstream &ofs, std::string file_path)
    {
    }

} // namespace multisensor_localization