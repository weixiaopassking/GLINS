#include "./imageProcess.hpp"

/**
 * @brief  相机去畸变
 *
 * */
void imageProcess::configParam(const YAML::Node config_node)
{
  /*1--读取相机类型*/
  _camera_type = config_node["camera_type"].as<std::string>();

  std::cout << "~~~~~~~~" << std::endl;
  /*2--读取相机内参数*/
  _camera_intrinsics.fx = config_node[_camera_type]["cam_intrinsics_param"]["fx"].as<double>();
  _camera_intrinsics.fy = config_node[_camera_type]["cam_intrinsics_param"]["fy"].as<double>();
  _camera_intrinsics.cx = config_node[_camera_type]["cam_intrinsics_param"]["cx"].as<double>();
  _camera_intrinsics.cy = config_node[_camera_type]["cam_intrinsics_param"]["cy"].as<double>();

  /*2--读取畸变系数*/
  if (_camera_type == "mono_pinhole")
  {
    _pinhole_distortion_coefficient.k1 = config_node[_camera_type]["camera_distortion_coefficient"]["k1"].as<double>();
    _pinhole_distortion_coefficient.k2 = config_node[_camera_type]["camera_distortion_coefficient"]["k2"].as<double>();
    _pinhole_distortion_coefficient.p1 = config_node[_camera_type]["camera_distortion_coefficient"]["p1"].as<double>();
    _pinhole_distortion_coefficient.p2 = config_node[_camera_type]["camera_distortion_coefficient"]["p2"].as<double>();
  }
  if (_camera_type == "mono_fisheye")
  {
    _fisheye_distortion_coefficient.k1 = config_node[_camera_type]["camera_distortion_coefficient"]["k1"].as<double>();
    _fisheye_distortion_coefficient.k2 = config_node[_camera_type]["camera_distortion_coefficient"]["k2"].as<double>();
    _fisheye_distortion_coefficient.k3 = config_node[_camera_type]["camera_distortion_coefficient"]["k3"].as<double>();
    _fisheye_distortion_coefficient.k4 = config_node[_camera_type]["camera_distortion_coefficient"]["k4"].as<double>();
  }

  /*3--读取特殊参数*/
  if (_camera_type == "stereo")
  {
    _camera_disparity = config_node[_camera_type]["cam_disparity"].as<double>();
  }
}
/**
 * 去畸变
 * */
cv::Mat imageProcess::execUndistort(cv::Mat image)
{
  const double rows = image.rows;
  const double cols = image.cols;

  cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1); // opencv [y,x]

  for (double u_index = 0; u_index < cols; u_index++)
  {
    for (double v_index = 0; v_index < rows; v_index++)
    {
      /*1--反向投影像素到归一化平面 z=1*/
      double x = (u_index - _camera_intrinsics.cx) / _camera_intrinsics.fx;
      double y = (v_index - _camera_intrinsics.cy) / _camera_intrinsics.fy;
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
      double u_distorted = _camera_intrinsics.fx * x_distorted + _camera_intrinsics.cx;
      double v_distorted = _camera_intrinsics.fy * y_distorted + _camera_intrinsics.cy;
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

  return image_undistorted;
}
