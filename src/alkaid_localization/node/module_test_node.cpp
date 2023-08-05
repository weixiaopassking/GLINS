#include "../data/geometry_data.hpp"
#include "../module/cloud_registration/cloud_registration_interface.hpp"
#include "../module/cloud_registration/ndt_registration.hpp"
#include "../module/cloud_registration/icp_registration.hpp"
#include "../module/cloud_filter/voxel_filter.hpp"
#include "../module/cloud_filter/cloud_filter_interface.hpp"
#include "../tools/tools.hpp"

#include <fstream>
#include <gtest/gtest.h> //unit
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>



/* time cost for registration of cloud point */
TEST(cloud_registration, instance1)
{
    std::cout << "[Test]$ unit test for module" << std::endl;
    /*1--get pointcloud and gt*/
    std::string resource_file_path = ros::package::getPath("alkaid_localization") + "/resource/";
    std::ifstream gt_stream(resource_file_path + "EPFL/aquarius_pose.txt");
    data_ns::Mat4f gt_maxtrix = data_ns::Mat4f::Identity();
    if (gt_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        gt_stream.close();

        gt_maxtrix.block<3, 3>(0, 0) = data_ns::Quatf(qw, qx, qy, qz).toRotationMatrix();
        gt_maxtrix.block<3, 1>(0, 3) << tx, ty, tz;
        std::cout << "gt_maxtrix:" << std::endl << gt_maxtrix << std::endl;
    }

    
    data_ns::CloudData::CLOUD_PTR source_cloud_ptr(new data_ns::CloudData::CLOUD);
    data_ns::CloudData::CLOUD_PTR target_cloud_ptr(new data_ns::CloudData::CLOUD);

    pcl::io::loadPCDFile(resource_file_path + "EPFL/aquarius_source.pcd", *source_cloud_ptr);
    pcl::io::loadPCDFile(resource_file_path + "EPFL/aquarius_target.pcd", *target_cloud_ptr);
    /*2--create method*/
    std::shared_ptr<module_ns::CloudRegistrationInterface> registration_ptr =
        std::make_shared<module_ns::ICPRegistration>();
    std::shared_ptr<module_ns::CloudFilterInterface> filter_ptr = std::make_shared<module_ns::VoxelFilter>(0.1);
    filter_ptr->Filter(source_cloud_ptr, source_cloud_ptr);
    filter_ptr->Filter(target_cloud_ptr, target_cloud_ptr);
    /*3-compare time*/
    TimeCost(
        [&]() {
    registration_ptr->SetTargetCloud(target_cloud_ptr);
    registration_ptr->SetSourceCloud(source_cloud_ptr);
    data_ns::Mat4f res_matrix = registration_ptr->GetResTransform(data_ns::Mat4f::Identity());
    std::cout << "res_matrix: " <<std::endl<< res_matrix << std::endl;
        },
        "classic icp", 1);

    registration_ptr = std::make_shared<module_ns::NDTRegistration>();
    TimeCost(
        [&]() {
            registration_ptr->SetTargetCloud(target_cloud_ptr);
            registration_ptr->SetSourceCloud(source_cloud_ptr);
            data_ns::Mat4f res_matrix = registration_ptr->GetResTransform(data_ns::Mat4f::Identity());
            std::cout << "res_matrix: " << std::endl << res_matrix << std::endl;
        },
        "classic ndt", 1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}