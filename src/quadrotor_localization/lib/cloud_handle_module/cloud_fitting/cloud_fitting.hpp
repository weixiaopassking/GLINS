static bool PlaneFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 4, 1> &plane_coeffs,
                         const double eps = 1e-2); //?
static bool LineFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 3, 1> &start_point,
                        Eigen::Matrix<double, 3, 1> &direction, const double eps = 0.2); //?