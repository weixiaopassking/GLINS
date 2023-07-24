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
    kdtree.setInputCloud(*_target_cloud_ptr);

    for (int iter = 0; iter < options_.max_iteration; ++iter)
    {
        std::for_each(std::execution::par_unseq, index_map.begin(), index_map.end(), [](int index) {
            Eigen::Vector3d q = _source_cloud_ptr->points[idx];
            Eigen::Vector3d q_r = res_transform * q; // 旋转过去
            pcl::PointXYZI search_point(q_r.x(),q_r.y(),q_r.z());
            // kdtree.nearestKSearch();
        });
    }
}

ICPRegistration::~ICPRegistration()
{
}