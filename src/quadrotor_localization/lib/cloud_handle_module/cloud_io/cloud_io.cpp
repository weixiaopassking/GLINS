#include "./cloud_io.hpp"

CloudIO::CloudIO()
{
    _cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

bool CloudIO::LoadCloud(const std::string pcd_path)
{
    if (pcd_path.empty())
    {
        ErrorAssert(demo);
    }
    return true;
}

bool CloudIO::LoadParam(const YAML::Node node)
{

    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIO::GetCloud()
{
    // 移动
}

bool CloudIO::SaveCloud()
{
    return true;
}

bool CloudIO::SaveParam()
{
    return true;
}

CloudIO ::~CloudIO()
{
}

//     // pcl::io::loadPCDFile(pcd_path, *_cloud_ptr);

//     // if (_cloud_ptr->empty())
//     // {
//     //     std::cout << "输入点云无效" << std::endl;
//     //     exit(0);
//     // }
//     // else
//     // {
//     //     std::cout << "点云数据规模为:" << _cloud_ptr->points.size() << std::endl;
//     // }
// }