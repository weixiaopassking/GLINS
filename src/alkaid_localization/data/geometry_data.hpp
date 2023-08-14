/*
 * @Description: definition for point cloud
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-07
 * @Note:  use float prio preferentially
 */

#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace data_ns
{
using Quatf = Eigen::Quaternionf;
// using Quatd = Eigen::Quaterniond;

using Vec3f = Eigen::Vector3f;
// using Vec3d = Eigen::Vector3d;

using Mat3f = Eigen::Matrix3f;

using Mat4f = Eigen::Matrix4f;
// using Mat4f = Eigen::Matrix4f;

using Mat6f = Eigen::Matrix6f;
// using Mat6f = Eigen::Matrix6f;

using Mat9f = Eigen::Matrix9f;
// using Mat6f = Eigen::Matrix6f;

using SE3f = Sophus::SE3f;
// using SE3d= Sophus::SE3d;

using SO3f = Sophus::SO3f;
// using SO3f= Sophus::SO3f;

} // namespace data_ns