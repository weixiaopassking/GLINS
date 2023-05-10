#include "./imageUndistort.hpp"

/**
 *
 *
 * */
imageUndistort::imageUndistort(cv::Mat image, const YAML::Node config_node)
{
  //todo fisheye
  // Pinhole camera model
  double k1 = config_node["camera_distorted"]["k1"].as<double>();
  double k2 = config_node["camera_distorted"]["k2"].as<double>();
  double p1 = config_node["camera_distorted"]["p1"].as<double>();
  double p2 = config_node["camera_distorted"]["p2"].as<double>();
  Eigen::Matrix3d camera_param = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      config_node["camera_Intrinsics"].as<std::vector<double>>().data());
  double fx = camera_param(0, 0);
  double cx = camera_param(0, 2);
  double fy = camera_param(1, 1);
  double cy = camera_param(1, 2);

  double rows = image.rows;
  double cols = image.cols;

  cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1); //opencv [y,x]
  for (double v_index = 0; v_index < rows; v_index++)
  {
    for (double u_index = 0; u_index < cols; u_index++)
    {
      double x = (u_index - cx) / fx;
      double y = (v_index - cy) / fy;
      double rr = x * x + y * y;
      double x_distorted = x * (1 + k1 * rr + k2 * rr * rr) + 2 * p1 * x * y + p2 * (rr + 2 * x * x);
      double y_distorted = y * (1 + k1 * rr + k2 * rr * rr) + 2 * p2 * y * x + p1 * (rr + 2 * y * y);

      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

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
  std::cout << "get " << std::endl;
}