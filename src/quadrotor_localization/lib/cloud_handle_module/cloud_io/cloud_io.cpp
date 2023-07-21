/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    cloud_io.hpp
*  @brief  点云的读写
*  @author  robotics gang
*  @date    2023/7/18
*  @version v0.1
*  @ref
****************************************************************************
*/

#include "./cloud_io.hpp"

/**
 * @brief  CloudIO构造函数
 * @param none
 * @return none
 * @note
 */
CloudIO::CloudIO()
{
    _cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

/**
 * @brief  pcd中路径读取点云
 * @param pcd_path pcd路径
 * @return bool
 * @note
 */
bool CloudIO::LoadCloud(const std::string pcd_path)
{
    if (pcd_path.empty())
    {
        ErrorAssert(ErrorCode::error_path, __FILE__, __FUNCTION__, __LINE__);
    }

    pcl::io::loadPCDFile(pcd_path, *_cloud_ptr);

    if (_cloud_ptr->empty())
    {
        ErrorAssert(ErrorCode::error_path, __FILE__, __FUNCTION__, __LINE__);
    }
    else
    {
        std::cout << "点云数据规模为:" << _cloud_ptr->points.size() << std::endl;
    }

    return true;
}

// todo
bool CloudIO::LoadParam(const YAML::Node node)
{

    return true;
}

/**
 * @brief  获取点云
 * @param
 * @return Ptr 点云指针
 * @note
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIO::GetCloud()
{
    if (_cloud_ptr->empty())
    {
        ErrorAssert(ErrorCode::error_file, __FILE__, __FUNCTION__, __LINE__);
    }
    return _cloud_ptr; // 可能有异常
}

// todo
bool CloudIO::SaveCloud()
{
    return true;
}

// todo
bool CloudIO::SaveParam()
{
    return true;
}

CloudIO ::~CloudIO()
{
}

std::ostream &operator<<(std::ostream &o, const CloudIO &obj)
{
    if (obj._cloud_ptr->empty())
    {
        ErrorAssert(ErrorCode::error_file, __FILE__, __FUNCTION__, __LINE__);
    }
    o << "检测" << std::endl;
    return o;
}