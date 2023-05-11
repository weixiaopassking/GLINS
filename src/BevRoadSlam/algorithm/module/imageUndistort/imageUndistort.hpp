#ifndef _IMAGE_UNDISTORT_HPP
#define _IMAGE_UNDISTORT_HPP
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class imageUndistort
{
public:
  imageUndistort() {};
  void configParam(const YAML::Node config_node); // 配置去畸变参数
  cv::Mat execUndistort(cv::Mat);                 // 执行去畸变
  ~imageUndistort(){};

private:
  /*相机内参数*/
  struct
  {
    double fx, fy, cx, cy;
  } _camera_Intrinsics;

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

  std::string  _camera_type;
};

#endif