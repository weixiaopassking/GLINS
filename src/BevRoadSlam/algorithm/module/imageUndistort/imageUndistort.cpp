#include "./imageUndistort.hpp"

/**
 * @brief  相机去畸变
 *
 * */
void imageUndistort::configParam(const YAML::Node config_node)
{
  /*1--读取相机内参*/
  Eigen::Matrix3d camera_param = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      config_node["camera_Intrinsics"].as<std::vector<double>>().data());
  _camera_Intrinsics.fx = camera_param(0, 0);
  _camera_Intrinsics.cx = camera_param(0, 2);
  _camera_Intrinsics.fy = camera_param(1, 1);
  _camera_Intrinsics.cy = camera_param(1, 2);
  /*2--读取相机类型及对应畸变系数*/
  _camera_type = config_node["camera_type"].as<std::string>();
  if (_camera_type == "pinhole")
  {
    _pinhole_distortion_coefficient.k1 = config_node["camera_distortion_coefficient"]["pinhole"]["k1"].as<double>();
    _pinhole_distortion_coefficient.k2 = config_node["camera_distortion_coefficient"]["pinhole"]["k2"].as<double>();
    _pinhole_distortion_coefficient.p1 = config_node["camera_distortion_coefficient"]["pinhole"]["p1"].as<double>();
    _pinhole_distortion_coefficient.p2 = config_node["camera_distortion_coefficient"]["pinhole"]["p2"].as<double>();
    std::cout << "-----------" << std::endl;
  }
  else if (_camera_type == "fisheye")
  {
    _fisheye_distortion_coefficient.k1 = config_node["camera_distortion_coefficient"]["fisheye"]["k1"].as<double>();
    _fisheye_distortion_coefficient.k2 = config_node["camera_distortion_coefficient"]["fisheye"]["k2"].as<double>();
    _fisheye_distortion_coefficient.k3 = config_node["camera_distortion_coefficient"]["fisheye"]["k3"].as<double>();
    _fisheye_distortion_coefficient.k4 = config_node["camera_distortion_coefficient"]["fisheye"]["k4"].as<double>();
  }
  else
  {
    std::cout << "暂时无此相机的畸变参数" << std::endl;
  }
}

cv::Mat imageUndistort::execUndistort(cv::Mat image)
{
  const double rows = image.rows;
  const double cols = image.cols;

  cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1); // opencv [y,x]

  for (double u_index = 0; u_index < cols; u_index++)
  {
    for (double v_index = 0; v_index < rows; v_index++)
    {
      /*1--反向投影像素到归一化平面*/
      double x = (u_index - _camera_Intrinsics.cx) / _camera_Intrinsics.fx;
      double y = (v_index - _camera_Intrinsics.cy) / _camera_Intrinsics.fy;
      double x_distorted, y_distorted;
      double rr = x * x + y * y;

      /*2--计算畸变坐标*/
      if (_camera_type == "pinhole")
      {
        x_distorted = x * (1 + _pinhole_distortion_coefficient.k1 * rr + _pinhole_distortion_coefficient.k2 * rr * rr) + 2 * _pinhole_distortion_coefficient.p1 * x * y + _pinhole_distortion_coefficient.p2 * (rr + 2 * x * x);
        y_distorted = y * (1 + _pinhole_distortion_coefficient.k1 * rr + _pinhole_distortion_coefficient.k2 * rr * rr) + 2 * _pinhole_distortion_coefficient.p2 * y * x + _pinhole_distortion_coefficient.p1 * (rr + 2 * y * y);
      }
      else if (_camera_type == "fisheye")
      {
        double theta = atan(sqrt(rr));
        double theta_distorted = theta * (1 + _fisheye_distortion_coefficient.k1 * pow(theta, 2) + _fisheye_distortion_coefficient.k2 * pow(theta, 4) + _fisheye_distortion_coefficient.k3 * pow(theta, 6) + _fisheye_distortion_coefficient.k4 * pow(theta, 8));
        x_distorted = theta_distorted / sqrt(rr) * x;
        y_distorted = theta_distorted / sqrt(rr) * y;
      }

      /*3--重新投影至像素坐标*/
      double u_distorted = _camera_Intrinsics.fx * x_distorted + _camera_Intrinsics.cx;
      double v_distorted = _camera_Intrinsics.fy * y_distorted + _camera_Intrinsics.cy;
      if (u_distorted >= 0 and v_distorted >= 0 and u_distorted < cols and v_distorted < rows)
      {
        image_undistorted.at<uchar>(v_index, u_index) = image.at<uchar>((int)v_distorted, (int)u_distorted);
      }
      else
      {
        image_undistorted.at<uchar>(v_index, u_index) = 0; // 黑
      }
    }
  }
  cv::imshow("校正后", image_undistorted);
  cv::waitKey(0);

  return image_undistorted;
}
