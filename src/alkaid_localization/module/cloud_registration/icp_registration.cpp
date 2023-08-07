#include "icp_registration.hpp"

namespace module_ns
{

/**
 * @brief   icp init
 * @param
 * @note
 **/
ICPRegistration::ICPRegistration()
{
    _source_cloud_ptr.reset(new data_ns::CloudData::CLOUD);
    _target_cloud_ptr.reset(new data_ns::CloudData::CLOUD);
    _has_gt_transform = false;
}

/**
 * @brief   icp get source cloud
 * @param
 * @note
 **/
void ICPRegistration::SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr)
{
  _source_cloud_ptr = source_cloud_ptr;
  _source_center_vec =
      std::accumulate(source_cloud_ptr->points.cbegin(), source_cloud_ptr->points.cend(), data_ns::Vec3f::Zero().eval(),
                      [](const data_ns::Vec3f &sum, const data_ns::CloudData::POINT &point) {
                          return sum + point.getVector3fMap();
                      }) /
      source_cloud_ptr->points.size();
}

/**
 * @brief  icp get target cloud
 * @param
 * @note
 **/
void ICPRegistration::SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr)
{
      _target_cloud_ptr = target_cloud_ptr;
      _target_center_vec = std::accumulate(target_cloud_ptr->points.cbegin(), target_cloud_ptr->points.cend(),
                                           data_ns::Vec3f::Zero().eval(),
                                           [](const data_ns::Vec3f &sum, const data_ns::CloudData::POINT &point) {
                                               return sum + point.getVector3fMap();
                                           }) /
                           target_cloud_ptr->points.size();
}

/**
 * @brief  icp set ground truth
 * @param
 * @note
 **/
void ICPRegistration::SetGtTransform(const data_ns::Mat4f &gt_transform)
{
      _gt_transform.matrix()=gt_transform;
      _has_gt_transform = true;
}

/**
 * @brief  icp get result
 * @param
 * @note
 **/
data_ns::Mat4f ICPRegistration::GetResTransform(const data_ns::Mat4f &predict_transform)
{
      if (_option.icp_method == ICPMethods::point2point)
      {
          return Point2Point(predict_transform);
    }
    else if (_option.icp_method == ICPMethods::point2line)
    {
          return data_ns::Mat4f::Identity();
    }
    else if (_option.icp_method == ICPMethods::point2lane)
    {
          return data_ns::Mat4f::Identity();
    }
    else
    {
          tools_ns::ErrorAssert("no mathcing  icp's methods", __FILE__, __FUNCTION__, __LINE__);
          return data_ns::Mat4f::Identity();//just for no warning but no use
    }

}

/**
 * @brief  icp sub method point2point
 * @param
 * @note
 **/
data_ns::Mat4f  ICPRegistration::Point2Point(const data_ns::Mat4f &predict_transform)
{

    /*1--check source cloud and target cloud*/
    if(_source_cloud_ptr==nullptr&&_target_cloud_ptr==nullptr)
    {
          tools_ns::ErrorAssert("icp registration is incorrect",__FILE__, __FUNCTION__, __LINE__);
    }
    /*2--prepare variable*/
    data_ns::SE3f res_se3_transform(predict_transform.block<3, 3>(0, 0), predict_transform.block<3, 1>(0, 3));

    if (!_option.use_initial_translation)
    {
          res_se3_transform.translation() = _target_center_vec - _source_center_vec;
    }
    std::vector<int> index_map_vec(_source_cloud_ptr->points.size()); 
    std::iota(index_map_vec.begin(), index_map_vec.end(), 0);//create index from zero
    std::vector<Eigen::Matrix<float, 6, 3>> jacobians_vec(index_map_vec.size()); // jacobi  succession
    std::vector<bool> effect_points_vec(index_map_vec.size(), false);             // effective points succession
    std::vector<Eigen::Vector3f> residual_vec(index_map_vec.size());              // residual succession

    /*2--build search tree*/
    pcl::KdTreeFLANN<data_ns::CloudData::POINT> kdtree;
    kdtree.setInputCloud(_target_cloud_ptr);

    /*3--gauss-Newton iteration*/
    for (int iter = 0; iter < _option.max_iteration; ++iter)
     {
          /*3.1--search nearest neighbour and calculate jacobi residual effectiva points*/
          std::for_each(std::execution::par_unseq, index_map_vec.begin(), index_map_vec.end(), [&](int index) 
          {

              data_ns::Vec3f q = _source_cloud_ptr->points[index].getVector3fMap();
              data_ns::Vec3f q_rotated = res_se3_transform * q;
              data_ns::CloudData::POINT search_point(q_rotated.x(), q_rotated.y(), q_rotated.z()); 

              std::vector<int> nn_search_index_vec(1);
              std::vector<float> nn_search_dis_vec(1);
              kdtree.nearestKSearch(search_point, 1, nn_search_index_vec, nn_search_dis_vec);

              if (!nn_search_index_vec.empty())
              {
                  Eigen::Vector3f p = _target_cloud_ptr->points[nn_search_index_vec.front()].getVector3fMap();
                  float dis_square = (p - q_rotated).squaredNorm();
                  if (dis_square > _option.max_point2point_distance)
                  {
                      effect_points_vec[index] = false;
                      return; // failed
                  }

                  Eigen::Vector3f residual = p - q_rotated; // e_i=p_i-Rq_i-t=p_i-q'_i
                  Eigen::Matrix<float, 6, 3> jacobian; // derivative of R t
                  jacobian.block<3, 3>(0, 0) = (res_se3_transform.so3().matrix() * Sophus::SO3f::hat(q))
                                                   .transpose();                         // derivative of R 3rows6cols
                  jacobian.block<3, 3>(3, 0) = (-1)*Eigen::Matrix3f::Identity().transpose(); // derivative of R t
                  jacobians_vec[index] = jacobian;
                  residual_vec[index] = residual;
                  effect_points_vec[index] = true;
              }
              else // nn_search_index_vec is empty
              {
                  effect_points_vec[index] = false;
              }
          }); // for_each

          /*3.2--accumulate  H_g and residual error   (Hdx=g)*/
          float residual_sum = 0;
          int effect_cnt = 0;
          auto H_g = std::accumulate(index_map_vec.begin(), index_map_vec.end(),
                                     std::pair<H_type, g_type>(H_type::Zero(), g_type::Zero()),
                                     [&](const std::pair<H_type, g_type> &pre, int index) -> std::pair<H_type, g_type> {
                                         if (!effect_points_vec[index])
                                         {
                                             return pre;
                                         }
                                         else
                                         {
                                             residual_sum += residual_vec[index].dot(residual_vec[index]);//e(x)^2
                                             effect_cnt++;
                                             return std::pair<H_type, g_type>(
                                                 pre.first + jacobians_vec[index] * jacobians_vec[index].transpose(),//J*J^T
                                                 pre.second - jacobians_vec[index] * residual_vec[index]);//-Je(x)
                                         }
                                     });
          if (effect_cnt < _option.min_nn_numbers)
          {
              tools_ns::ErrorAssert("too few effect points ", __FILE__, __FUNCTION__, __LINE__);
         }

        /*3.3.   calculate  dx*/
        H_type H = H_g.first;
        g_type g = H_g.second;
        dx_type dx = H.inverse() * g;

        /*3.4.update error by dx*/
        res_se3_transform.so3() = res_se3_transform.so3() * Sophus::SO3f::exp(dx.head<3>());
        res_se3_transform.translation() += dx.tail<3>();

        /*3.5. check out if convergence*/
        if (dx.norm() < _option.epsilon)
        {
              if (_has_gt_transform == true)
              {
                  float transform_error = (_gt_transform.inverse() * res_se3_transform).log().norm();
                  tools_ns::VariableAssert("transform_error", transform_error);
              }
            break;
        }
    } // for (iter = 0: _options.max_iteration)

    /*4  return res*/
    return   res_se3_transform.matrix();

}


//todo
data_ns::Mat4f ICPRegistration::Point2Plane(const data_ns::Mat4f &predict_transform)
{
    return data_ns::Mat4f::Identity();
}
data_ns::Mat4f ICPRegistration::Point2Line(const data_ns::Mat4f &predict_transform)
{
    return data_ns::Mat4f::Identity();
}


 } // namespace module_ns
