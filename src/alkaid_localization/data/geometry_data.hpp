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
// using Mat4d = Eigen::Matrix4d;

using SE3f = Sophus::SE3f;
// using SE3d= Sophus::SE3d;

} // namespace data_ns