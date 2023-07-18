#include "./cloud_io.hpp"

/**
 * @brief  点云数据读取
 * @param source_path source点云 pcd的路径
 * @return
 * @note
 */

bool CloudIO::LoadPcd(const std::string pcd_path)
{
    // _cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    // if (pcd_path.empty())
    // {
    //     std::cout << "输入点云路径错误" << std::endl;
    //     exit(0);
    // }

    // pcl::io::loadPCDFile(pcd_path, *_cloud_ptr);

    // if (_cloud_ptr->empty())
    // {
    //     std::cout << "输入点云无效" << std::endl;
    //     exit(0);
    // }
    // else
    // {
    //     std::cout << "点云数据规模为:" << _cloud_ptr->points.size() << std::endl;
    // }
}

bool CloudIO::SavePcd()
{
    return true;
}
