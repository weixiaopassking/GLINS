#include "./imageUndistort.hpp"

/**
 *
 *
 * */
imageUndistort::imageUndistort(cv::Mat image, const YAML::Node config_node) {
  double k1 = config_node["camera_distorted"]["k1"].as<double>();
  double k2 = config_node["camera_distorted"]["k2"].as<double>();
  double p1 = config_node["camera_distorted"]["p1"].as<double>();
  double p2 = config_node["camera_distorted"]["p2"].as<double>();
  Eigen::Matrix3d camera_param =
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          config_node["camera_Intrinsics"].as<std::vector<double>>().data());
  double fx = camera_param(0, 0);
  double cx = camera_param(0, 2);
  double fy = camera_param(1, 1);
  double cy = camera_param(1, 2);

  double rows = image.rows;
  double cols = image.cols;
  auto lambda_rr = [](double v, double u) -> double {
    return pow(v, 2) + pow(u, 2);
  };
  cv::Mat image_undistorted = cv::Mat(rows, cols, CV_8UC1);//[y,x]
  for (double v_index = 0; v_index < rows; v_index++) {
    for (double u_index = 0; u_index < cols; u_index++) {
      double u_distorted = 0, v_distorted = 0;

     double u = (u_index - cx) / fx;
     double  v = (v_index - cy) / fy;
     double rr = lambda_rr(v,u);
     u_distorted = u * (1 + k1 * rr + k2 * pow(rr, 2)) + 2 * p1 * v * u +
                   p2 * (rr + 2 * pow(u, 2));
     v_distorted = v * (1 + k1 * rr + k2 * pow(rr, 2)) + 2 * p2 * v * u +
                   p1 * (rr + 2 * pow(v, 2));

     u_distorted = fx* u_distorted + cx;
     v_distorted = fy * v_distorted + cy;

     if (u_distorted >= 0 and v_distorted >= 0 and u_distorted < cols and
         v_distorted < rows) {
       image_undistorted.at<uchar>(v_index, u_index) =
           image.at<uchar>((int)v_distorted, (int)u_distorted);
      } else {
        image_undistorted.at<uchar>(v_index, u_index) = 0;  // 黑
      }
    }
  }

  cv::imshow("校正后", image_undistorted);
  cv::waitKey();
  std::cout << "get " << std::endl;
  // std::cout << "k1:" << k1 << std::endl;
  // std::cout << "k2:" << k2 << std::endl;
  // std::cout << "fx:" << fx << std::endl;
  // std::cout << "fy:" << fy << std::endl;
  // std::cout << "cx:" << cx << std::endl;
  // std::cout << "cy:" << cy << std::endl;
  // std::cout << "p1:" << p1 << std::endl;
  // std::cout << "p2:" << p2 << std::endl;
}