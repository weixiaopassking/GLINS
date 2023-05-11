/*
 * @Description: 测试节点
 * @Function:
 * @Author: gang
 * @Version : v1.0
 * @Date: 2023-05-07
 */

// ros库
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
// 系统库
#include <memory>
#include <string>
// 第三方库
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
// 自定义库
#include "../algorithm/module/imageProcess/imageProcess.hpp"
#include "../algorithm/tools/paramParseUtils.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ndoe");
    ros::NodeHandle nh;

    const std::string package_path = ros::package::getPath("BevRoadSlam");
    YAML::Node camera_config_node = YAML::LoadFile(package_path + "/config/camera.yaml");
    std::string image_path = package_path + "/data/pinhole.png";

    cv::Mat image = cv::imread(image_path,cv::IMREAD_GRAYSCALE);
    // cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

    cv::imshow("原始图像", image);
    cv::waitKey(0);
    std::unique_ptr<imageProcess> image_ptr = std::make_unique<imageProcess>();
    image_ptr->configParam(camera_config_node);
    image = image_ptr->execUndistort(image).clone();
    cv::imshow("校正图像", image);
    cv::waitKey(0);
    return 0;
}
