#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "../algorithm/module/imageUndistort/imageUndistort.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_ndoe");
  ros::NodeHandle nh;
  YAML::Node config_node_ = YAML::LoadFile(
      "/home/g/workspace/BEV-Semantic-Slam-for-Parking/src/avp/config/camera.yaml");
  const std::string image_path =
      "/home/g/workspace/BEV-Semantic-Slam-for-Parking/src/avp/data/test.png";
  cv::Mat image = cv::imread(image_path, 0);  // 灰度读入
  cv::imshow("demo", image);
  cvWaitKey();
  std::unique_ptr<imageUndistort> demo_ptr =
      std::make_unique<imageUndistort>(image,config_node_);
  return 0;
}
