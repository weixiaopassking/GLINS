#ifndef _IMAGE_PROCESS_HPP
#define _IMAGE_PROCESS_HPP
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class imageProcess
{
public:
  imageProcess(){};
  void configParam(const YAML::Node config_node); // 配置去畸变参数
  cv::Mat execUndistort(cv::Mat image);           // 去畸变
  void execPointCloud(cv::Mat left_image, cv::Mat right_image);
  ~imageProcess(){};

private:
  /*相机内参数*/
  struct
  {
    double fx, fy, cx, cy;
  } _camera_intrinsics;

  /*针孔畸变参数*/
  struct
  {
    double k1, k2, p1, p2;
  } _pinhole_distortion_coefficient;
  /*鱼眼畸变参数*/
  struct
  {
    double k1, k2, k3, k4;

  } _fisheye_distortion_coefficient;

  double _camera_disparity; // 仅针对双目相机

  std::string _camera_type;
};

#endif