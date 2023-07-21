#include "../../common/project_path.h" //工程全局路径
#include "../../common/debug_info.hpp" //工程全局路径

#include "../lib/cloud_handle_module/cloud_filter/cloud_filter_interface.hpp"
#include "../lib/cloud_handle_module/cloud_filter/voxel_filter/voxel_filter.hpp"
#include "../lib/cloud_handle_module/cloud_io/cloud_io.hpp"
#include "../lib/cloud_handle_module/cloud_io/cloud_io.hpp"

#include <gtest/gtest.h>     //单元测试
#include <memory>            //智能指针
#include <pcl/point_cloud.h> //点云
#include <pcl/point_types.h> //点

#if 0
TEST(cloud_handle_module, baic_use)
{
    const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
    std::shared_ptr<CloudIO> cloud_io_ptr = std::make_shared<CloudIO>();
    cloud_io_ptr->LoadCloud(data_file_path + "map_example.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = cloud_io_ptr->GetCloud();
    std::cout << "获取到的点云尺寸:" << cloud_ptr->size() << std::endl;
    /*滤波*/
    std::shared_ptr<CloudFilterInterface> cloud_filter_ptr = std::make_shared<VoxelFilter>(0.5, 0.5, 0.5);
    cloud_filter_ptr->Filter(cloud_ptr, cloud_ptr);
    std::cout << "滤波1后的点云尺寸:" << cloud_ptr->size() << std::endl;
    cloud_filter_ptr = std::make_shared<VoxelFilter>(1, 1, 1);
    cloud_filter_ptr->Filter(cloud_ptr, cloud_ptr);
    std::cout << "滤波2后的点云尺寸:" << cloud_ptr->size() << std::endl;
    // CloudViewer::ViewerByPcl(cloud_ptr);
    // CloudViewer::ViewerByOpencv(cloud_ptr);
    SUCCEED();
}

#endif

TEST(cloud_handle_module, icp)
{
    const std::string data_file_path = static_cast<std::string>(PROJECT_PATH) + "/data/";
    std::ifstream gt_file_stream(data_file_path + "EPFL/kneeling_lady_pose.txt");
    if (gt_file_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_file_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        DebugInfo("tx", tx, "ty", ty, "tz", tz, "qw", qw, "qx", qx, "qy", qy, "qz", qz);
        gt_file_stream.close();
    }
    else
    {
            ErrorAssert(ErrorCode::error_file, __FILE__, __FUNCTION__, __LINE__);
    }

    SUCCEED();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
