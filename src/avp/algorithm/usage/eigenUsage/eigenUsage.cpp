#include "eigenUsage.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>
// 稠密矩阵的代数运算（逆，特征值等）

eigenUsage::eigenUsage() { this->solveLinearEquationsUse(); }

void eigenUsage::commonUse() {
  /*1--初始化、赋值与访问*/
  Eigen::Matrix<float, 2, 3> matrix_23;
  Eigen::Vector3d v_3d;
  Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;

  matrix_23 << 1, 2, 3, 4, 5, 6;
  std::cout << matrix_23 << std::endl;
  rrrr

      for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      std::cout << matrix_23(i, j) << " ";
    }
    std::cout << std::endl;
  }

  /*2--矩阵运算*/
  matrix_33 = Eigen::Matrix3d::Random();
  std::cout << matrix_33 << std::endl;
  std::cout << "转置:\t" << matrix_33.transpose() << std::endl;
  std::cout << "元素和:\t" << matrix_33.sum() << std::endl;
  std::cout << "迹:\t" << matrix_33.trace() << std::endl;
  std::cout << "逆:\t" << matrix_33.inverse() << std::endl;
}

// 求解线性方程组
void eigenDemo::solveLinearEquationsUse() {
  const int MATRIX_SIZE = 100;
  Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_nn;
  Eigen::Matrix<double, MATRIX_SIZE, 1> vec_n;

  matrix_nn = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  vec_n = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

  Eigen::Matrix<double, MATRIX_SIZE, 1> res_n;

  /*1--QR分解*/

  res_n = matrix_nn.colPivHouseholderQr().solve(vec_n);
  std::cout << res_n.transpose().size() << std::endl;

  /*2--Cholesky分解*/
  res_n = matrix_nn.llt().solve(vec_n);
  std::cout << res_n.transpose().size() << std::endl;

  /*3--svd分解*/
}
