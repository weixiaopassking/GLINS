#include "./icp_registration.hpp"

ICPRegistration::ICPRegistration()
{
    this->_source_cloud_ptr.reset();
    this->_target_cloud_ptr.reset();
    has_gt_transform = false;
}

void ICPRegistration::SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr)
{
    this->_source_cloud_ptr = source_cloud_ptr;
    this->_source_center_vec = std::accumulate(source_cloud_ptr->points.cbegin(), source_cloud_ptr->points.cend(),
                                               Eigen::Vector3d::Zero().eval(),
                                               [](const Eigen::Vector3d &sum, const pcl::PointXYZI &point) {
                                                   return sum + point.getVector3fMap().cast<double>();
                                               }) /
                               source_cloud_ptr->points.size();
    VariableInfo("source 点云几何中心", _source_center_vec.transpose());
}

void ICPRegistration::SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr)
{

    this->_target_cloud_ptr = target_cloud_ptr;
    this->_target_center_vec = std::accumulate(target_cloud_ptr->points.cbegin(), target_cloud_ptr->points.cend(),
                                               Eigen::Vector3d::Zero().eval(),
                                               [](const Eigen::Vector3d &sum, const pcl::PointXYZI &point) {
                                                   return sum + point.getVector3fMap().cast<double>();
                                               }) /
                               target_cloud_ptr->points.size();
    VariableInfo("target 点云几何中心", _target_center_vec.transpose());
}

void ICPRegistration::SetGtTransform(const Sophus::SE3d &gt_transform)
{

    this->_gt_transform = gt_transform;
    has_gt_transform = false;
    VariableInfo("配准点云的gt真值", gt_transform.matrix());
}

bool  ICPRegistration::GetResTransform(Sophus::SE3d &init_transform)
{

    VariableInfo("开始点云配准");
    /*1--点云检查*/
    if (_source_cloud_ptr == nullptr || _target_cloud_ptr == nullptr)
    {
        ErrorAssert(ErrorCode::error_file, __FILE__, __FUNCTION__, __LINE__);
    }

    /*2--变量准备*/
    Sophus::SE3d res_transform = init_transform;                      // 变换矩阵初始化
    std::vector<int> index_map_vec(_source_cloud_ptr->points.size()); // source 索引序列
    std::iota(index_map_vec.begin(), index_map_vec.end(),0);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians_vec(index_map_vec.size()); // 雅可比序列
    std::vector<bool> effect_points_vec(index_map_vec.size(), false); // 有效点序列
    std::vector<Eigen::Vector3d> residual_vec(index_map_vec.size());

    /*3--建立搜索树*/
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(_target_cloud_ptr);


    /*4--开始迭代*/
    for (int iter = 0; iter < _options.max_iteration; ++iter)
    {

        /*4.1--计算雅可比、残差、有效点*/
        std::for_each(std::execution::par_unseq, index_map_vec.begin(), index_map_vec.end(), [&](int index) {
            Eigen::Vector3d q = _source_cloud_ptr->points[index].getVector3fMap().cast<double>();
            Eigen::Vector3d q_r = res_transform * q; // 旋转过去
            pcl::PointXYZI search_point;
            search_point.x = q_r.x();
            search_point.y = q_r.y();
            search_point.z = q_r.z();

            std::vector<int> nn_search_index_vec(1);
            std::vector<float> nn_search_dis_vec(1);
            kdtree.nearestKSearch(search_point, 1, nn_search_index_vec, nn_search_dis_vec);

            if (!nn_search_index_vec.empty())
            {
                Eigen::Vector3d p =
                    _target_cloud_ptr->points[nn_search_index_vec.front()].getVector3fMap().cast<double>();
                double dis_square = (p - q_r).squaredNorm();
                if (dis_square > _options.max_point2point_distance)
                {
                    effect_points_vec[index] = false;
                    return; //?可能是出现了异常
                }

                /*构建残差*/
                Eigen::Vector3d residual = p - q_r;                               // e_i=p_i-Rq_i-t=p_i-q'_i
                Eigen::Matrix<double, 3, 6> J;                                    // 对R t分别求导
                J.block<3, 3>(0, 0) = res_transform.so3().matrix() * Sophus::SO3d::hat(q); // 对R的偏导数  3行6列
                J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();               // 对t偏导数
                jacobians_vec[index] = J;
                residual_vec[index] = residual;
                effect_points_vec[index] = true;
 
            }
            else // nn_search_index_vec is empty
            {
                effect_points_vec[index] = false;
            }

        }); // for_each

        // JJ^Tdeltax=—Jf(x)
        double sum_residuals = 0;
        int effect_num = 0;
        auto H_g =
            std::accumulate(index_map_vec.begin(), index_map_vec.end(),
                            std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>>(
                                Eigen::Matrix<double, 6, 6>::Zero(), Eigen::Matrix<double, 6, 1>::Zero()),
                            [&](const std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>> &pre,
                                int index) -> std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>> {
                                if (!effect_points_vec[index])
                                {
                                    return pre;
                                }
                                else
                                {
                                    sum_residuals += residual_vec[index].dot(residual_vec[index]); 
                                    effect_num++;
                                    return std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 1>>(
                                        pre.first + jacobians_vec[index].transpose() * jacobians_vec[index],
                                        pre.second - jacobians_vec[index].transpose() * residual_vec[index]);
                                }
                            });
        if (effect_num < _options.min_nn_numbers)
        {
            // warning
            return false;
        }
        /*计算dx*/
        Eigen::Matrix<double, 6, 6> H = H_g.first;
        Eigen::Matrix<double, 6, 1> g = H_g.second;
        Eigen::Matrix<double,6,1>dx=H.inverse()*g;
        /*更新fx*/
        res_transform.so3() = res_transform.so3() * Sophus::SO3d::exp(dx.head<3>());
        res_transform.translation() += dx.tail<3>();
        
        /*真值检查*/

        /*收敛判定*/
        if(dx.norm()<_options.epsilon)
        {
            VariableInfo("dx",dx);
            break;
        }
        std::cout<<std::endl;
        VariableInfo("收敛状态", dx.norm());

    } // for (iter = 0: _options.max_iteration)

    init_transform = res_transform;
    return true;
}

ICPRegistration::~ICPRegistration()
{
}