#ifndef _IMAGE_UNDISTORT_HPP
#define _IMAGE_UNDISTORT_HPP
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

class imageUndistort {
 public:
  imageUndistort() = delete;
  imageUndistort(cv::Mat image, const YAML::Node config_node);
  ~imageUndistort(){};
};

#endif