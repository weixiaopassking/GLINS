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

void ICPRegistration::GetResTransform(Sophus::SE3d &init_transform)
{
    VariableInfo("开始点云配准");
    /*1--点云检查*/
    if (_source_cloud_ptr == nullptr || _target_cloud_ptr == nullptr)
    {
        ErrorAssert(ErrorCode::error_path, __FILE__, __FUNCTION__, __LINE__);
    }

    Sophus::SE3d res_transform = init_transform;

    /*2--生成对点的索引*/
    std::vector<int> index_map(_source_cloud_ptr->points.size());
    std::iota(index_map.begin(), index_map.end(),
              _source_cloud_ptr->points.size()); // 源希腊字母  Ι ι aiot 约塔 微小，一点儿

    /*3--建立搜索树*/
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(_target_cloud_ptr);
    std::vector<bool> effect_points_flag(index_map.size(), false);

    for (int iter = 0; iter < _options.max_iteration; ++iter)
    {
        std::for_each(std::execution::par_unseq, index_map.begin(), index_map.end(), [&](int index) {
            Eigen::Vector3d q = _source_cloud_ptr->points[index].getVector3fMap().cast<double>();
            Eigen::Vector3d q_r = res_transform * q; // 旋转过去
            pcl::PointXYZI search_point;
            search_point.x = q_r.x();
            search_point.x = q_r.y();
            search_point.x = q_r.z();

            std::vector<int> nn_search_index_vec(1);
            std::vector<float> nn_search_dis_vec(1);
            kdtree.nearestKSearch(search_point, 1, nn_search_index_vec, nn_search_dis_vec);

            if (!nn_search_index_vec.empty())
            {
                Eigen::Vector3d p = _target_cloud_ptr->points[nn_search_index_vec.front()].getVector3fMap().cast<double>();
                double dis_square=(p-q_r).squaredNorm();
                if (dis_square > _options.max_point2point_distance)
                {
                    effect_points_flag[index] = false;
                    return ;//?
                }
                effect_points_flag[index] = true;

                /*构建残差*/
                Eigen::Vector3d residual=p-q_r;   //e_i=p_i-Rq_i-t=p_i-q'_i
                Eigen::Matrix<double,3,6> J;     //对R t分别求导
                J.block<3, 3>(0, 0) = res_transform.so3().matrix() * SO3::hat(q);//对R的偏导数
                J.block<3, 3>(0, 3)=-Eigen::Matrix3d::Identity();//对t偏导数
            }

        });
    }
}

ICPRegistration::~ICPRegistration()
{
}