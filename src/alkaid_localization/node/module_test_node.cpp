#include <fstream>
#include <gtest/gtest.h> //unit 
#include <iostream>
#include <ros/package.h>
#include "../data/geometry_data.hpp"

/* time cost for registration */
TEST(cloud_registration, instance1)
{
    std::cout << "[Test]$ unit test for module" << std::endl;
     std::string resource_file_path = ros::package::getPath("alkaid_localization") + "/resource/";
    std::cout << "[Test]$ resource  path is:" << resource_file_path << std::endl;
    std::ifstream gt_stream(resource_file_path + "EPFL/kneeling_lady_pose.txt");
    if(gt_stream.is_open())
    {
        double tx, ty, tz, qw, qx, qy, qz;
        gt_stream >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        gt_stream.close();
        
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}