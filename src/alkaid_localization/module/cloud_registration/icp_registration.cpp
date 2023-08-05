

// #include "icp_registration.hpp"

// namespace module_ns
// {

// ICPRegistration::ICPRegistration()
// {
//     this->_source_cloud_ptr.reset();
//     this->_target_cloud_ptr.reset();
//     _has_gt_transform = false;
// }

// // void ICPRegistration::SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr)
// // {
// //     this->_source_cloud_ptr = source_cloud_ptr;
// //     this->_source_center_vec = std::accumulate(source_cloud_ptr->points.cbegin(), source_cloud_ptr->points.cend(),
// //                                                Eigen::Vector3d::Zero().eval(),
// //                                                [](const Eigen::Vector3d &sum, const pcl::PointXYZI &point) {
// //                                                    return sum + point.getVector3fMap().cast<double>();
// //                                                }) /
// //                                source_cloud_ptr->points.size();
// //     // common_ns::VariableInfo("source 点云几何中心", _source_center_vec.transpose());
// // }

// // /**
// //  * @brief 设置目标点云
// //  * @details
// //  * @param target_cloud_ptr 目标点云指针
// //  * @return 无
// //  * @todo
// //  * @note
// //  **/
// // void ICPRegistration::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr)
// // {

// //     this->_target_cloud_ptr = target_cloud_ptr;
// //     this->_target_center_vec = std::accumulate(target_cloud_ptr->points.cbegin(), target_cloud_ptr->points.cend(),
// //                                                Eigen::Vector3d::Zero().eval(),
// //                                                [](const Eigen::Vector3d &sum, const pcl::PointXYZI &point) {
// //                                                    return sum + point.getVector3fMap().cast<double>();
// //                                                }) /
// //                                target_cloud_ptr->points.size();
// //     // common_ns::VariableInfo("target 点云几何中心", _target_center_vec.transpose());
// // }

// // /**
// //  * @brief 设置T变换真值
// //  * @details
// //  * @param gt_transform 李群SE3
// //  * @return 无
// //  * @note
// //  **/
// // void ICPRegistration::SetGtTransform(const Sophus::SE3d &gt_transform)
// // {

// //     this->_gt_transform = gt_transform;
// //     _has_gt_transform = true;
// // }

// // /**
// //  * @brief 得到变换结果
// //  * @details 为适配各方法的接口
// //  * @param init_transform 引用传出
// //  * @return 是否成功
// //  * @note
// //  **/
// // bool ICPRegistration::GetResTransform(Sophus::SE3d &init_transform)
// // {

// //     /*1--点云检查*/
// //     if (_source_cloud_ptr == nullptr || _target_cloud_ptr == nullptr)
// //     {
// //         common_ns::ErrorAssert("icp获取点云无效", __FILE__, __FUNCTION__, __LINE__);
// //     }

// //     /*2--icp配准方法选择*/
// //     if (_options.icp_method == ICPMethods::point2point)
// //     {
// //         // common_ns::VariableInfo("icp配准方法", "点到点");
// //         return Point2Point(init_transform);
// //     }
// //     else if (_options.icp_method == ICPMethods::point2line)
// //     {
// //         //   common_ns::VariableInfo("icp配准方法", "点到线"); // todo
// //         return false;
// //     }
// //     else if (_options.icp_method == ICPMethods::point2lane)
// //     {
// //         //  common_ns::VariableInfo("icp配准方法", "点到面"); // todo
// //         return false;
// //     }
// //     else
// //     {
// //         common_ns::ErrorAssert("不支持的icp方法", __FILE__, __FUNCTION__, __LINE__);
// //     }
// //     return false; // 不会跑到这里
// // }

// // /**
// //  * @brief icp point2point具体实现
// //  * @details
// //  * @param init_transform 引用传出
// //  * @return 是否成功
// //  * @note
// //  **/
// // bool ICPRegistration::Point2Point(Sophus::SE3d &init_transform)
// // {
// //     /*1--变量准备*/

// //     Sophus::SE3d res_transform = init_transform; // 变换矩阵初始化
// //     if (!_options.use_initial_translation)
// //     {
// //         res_transform.translation() = _target_center_vec - _source_center_vec;
// //     }
// //     std::vector<int> index_map_vec(_source_cloud_ptr->points.size()); // source点云索引序列
// //     std::iota(index_map_vec.begin(), index_map_vec.end(), 0);
// //     std::vector<Eigen::Matrix<double, 6, 3>> jacobians_vec(index_map_vec.size()); // 雅可比序列
// //     std::vector<bool> effect_points_vec(index_map_vec.size(), false);             // 有效点序列
// //     std::vector<Eigen::Vector3d> residual_vec(index_map_vec.size());              // 残差序列

// //     /*2--建立搜索树*/
// //     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
// //     kdtree.setInputCloud(_target_cloud_ptr);

// //     /*3--高斯牛顿迭代*/
// //     for (int iter = 0; iter < _options.max_iteration; ++iter)
// //     {

// //         std::for_each(std::execution::par_unseq, index_map_vec.begin(), index_map_vec.end(), [&](int index) {
// //             /*3.1--最近邻搜索*/
// //             Eigen::Vector3d q = _source_cloud_ptr->points[index].getVector3fMap().cast<double>();
// //             Eigen::Vector3d q_r = res_transform * q; // 旋转过去
// //             pcl::PointXYZI search_point;             // 当前搜索点
// //             search_point.x = q_r.x();
// //             search_point.y = q_r.y();
// //             search_point.z = q_r.z();

// //             std::vector<int> nn_search_index_vec(1);
// //             std::vector<float> nn_search_dis_vec(1);
// //             kdtree.nearestKSearch(search_point, 1, nn_search_index_vec, nn_search_dis_vec);

// //             if (!nn_search_index_vec.empty())
// //             {
// //                 Eigen::Vector3d p =
// //                     _target_cloud_ptr->points[nn_search_index_vec.front()].getVector3fMap().cast<double>();
// //                 double dis_square = (p - q_r).squaredNorm();
// //                 if (dis_square > _options.max_point2point_distance)
// //                 {
// //                     effect_points_vec[index] = false;
// //                     return; // 标记一下弃用该点
// //                 }

// //                 /*3.2--计算雅可比、残差、有效点标志位*/
// //                 Eigen::Vector3d residual = p - q_r;   // e_i=p_i-Rq_i-t=p_i-q'_i
// //                 Eigen::Matrix<double, 6, 3> jacobian; // 对R t分别求导
// //                 jacobian.block<3, 3>(0, 0) =
// //                     (res_transform.so3().matrix() * Sophus::SO3d::hat(q)).transpose(); // 对R的偏导数  3行6列
// //                 jacobian.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity().transpose(); // 对t偏导数
// //                 jacobians_vec[index] = jacobian;
// //                 residual_vec[index] = residual;
// //                 effect_points_vec[index] = true;
// //             }
// //             else // nn_search_index_vec is empty
// //             {
// //                 effect_points_vec[index] = false;
// //             }
// //         }); // for_each

// //         /*3.3--计算累计H和g Hdx=g*/

// //         double residual_sum = 0;
// //         int effect_cnt = 0;
// //         auto H_g = std::accumulate(index_map_vec.begin(), index_map_vec.end(),
// //                                    std::pair<H_type, g_type>(H_type::Zero(), g_type::Zero()),
// //                                    [&](const std::pair<H_type, g_type> &pre, int index) -> std::pair<H_type, g_type>
// //                                    {
// //                                        if (!effect_points_vec[index])
// //                                        {
// //                                            return pre;
// //                                        }
// //                                        else
// //                                        {
// //                                            residual_sum += residual_vec[index].dot(residual_vec[index]);
// //                                            effect_cnt++;
// //                                            return std::pair<H_type, g_type>(
// //                                                pre.first + jacobians_vec[index] * jacobians_vec[index].transpose(),
// //                                                pre.second - jacobians_vec[index] * residual_vec[index]);
// //                                        }
// //                                    });
// //         if (effect_cnt < _options.min_nn_numbers)
// //         {
// //             // warning
// //             return false;
// //         }
// //         /*3.4. 计算dx*/
// //         H_type H = H_g.first;
// //         g_type g = H_g.second;
// //         dx_type dx = H.inverse() * g;
// //         /*6.更新误差函数f(x)*/
// //         res_transform.so3() = res_transform.so3() * Sophus::SO3d::exp(dx.head<3>());
// //         res_transform.translation() += dx.tail<3>();

// //         /*3.5 真值检查*/
// //         if (_has_gt_transform == true)
// //         {
// //             double transform_error = (_gt_transform.inverse() * res_transform).log().norm(); //?
// //         }
// //         /*3.6 收敛判定*/
// //         if (dx.norm() < _options.epsilon)
// //         {
// //             break;
// //         }
// //     } // for (iter = 0: _options.max_iteration)
// //     /*4 结果返出*/
// //     init_transform = res_transform;
// //     return true;
// // }

// // // todo 以下功能
// // bool ICPRegistration::Point2Plane(Sophus::SE3d &init_transform)
// // {
// //     return true;
// // }
// // bool ICPRegistration::Point2Line(Sophus::SE3d &init_transform)
// // {
// //     return true;
// // }

// // ICPRegistration::~ICPRegistration()
// // {
// // }
// // } // namespace algorithm_ns