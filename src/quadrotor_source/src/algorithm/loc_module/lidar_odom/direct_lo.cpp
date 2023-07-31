#include "./direct_lo.hpp"

namespace algorithm_ns
{
DirectLo::DirectLo()
{
    /*1. 选择配准方法*/
    if (_options.registration_method == RegistrationMethods::ICP)
    {
        _cloud_registration_ptr = std::make_shared<algorithm_ns::ICPRegistration>();
    }
    else if (_options.registration_method == RegistrationMethods::NDT)
    {
        _cloud_registration_ptr = std::make_shared<algorithm_ns::NDTRegistration>();
    }
    else
    {
        common_ns::ErrorAssert("无对应的配置方法", __FILE__, __FUNCTION__, __LINE__);
    }
}

/**
 * @brief 更新位姿
 * @todo
 * @note
 **/
bool DirectLo::Update(pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr, Sophus::SE3d &current_pose)
{

     /*1--处理第一帧*/
    if (_local_map_ptr == nullptr)
    {
        _local_map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        (*_local_map_ptr) += (*scan_ptr);
        _cloud_registration_ptr->SetTargetCloud(_local_map_ptr);
        return true;
    }

    /*2--处理之后帧,计算R t*/
    _cloud_registration_ptr->SetSourceCloud(scan_ptr);
    if (_estimated_poses_vec.size() >= 2)
    {
        Sophus::SE3d T1 = _estimated_poses_vec[_estimated_poses_vec.size() - 1];
        Sophus::SE3d T2 = _estimated_poses_vec[_estimated_poses_vec.size() - 2];
        current_pose = T1 * (T2.inverse() * T1);
    }
    _cloud_registration_ptr->GetResTransform(current_pose);
    _estimated_poses_vec.emplace_back(current_pose);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_world_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*scan_ptr, *scan_world_ptr, current_pose.matrix().cast<float>()); // scan转至世界系

    /*3--p判断是否为关键帧*/
    if (IsKeyFrame(current_pose) == true)
    {
        _last_key_fram_pose = current_pose;
        /*3.1--维护FIFO*/
        _local_map_scans_que.emplace_back(scan_world_ptr);
        if (_local_map_scans_que.size() > _options.local_map_key_frames_num) // 维护一个FIFO
        {
            _local_map_scans_que.pop_front();
        }
        /*3.2--重建local_map*/
        _local_map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto &it_ptr : _local_map_scans_que)
        {
            *_local_map_ptr += *it_ptr;
        }
        _cloud_registration_ptr->SetTargetCloud(_local_map_ptr);
    }
}

bool DirectLo::SaveMap()
{
    return true;
}
DirectLo::~DirectLo()
{
}

/**
 * @brief 判断是否关键帧
 * @todo
 * @note
 **/
bool DirectLo::IsKeyFrame(const Sophus::SE3d &current_pose)
{
    Sophus::SE3d delta = _last_key_fram_pose.inverse() * current_pose;
    if (delta.translation().norm() > _options.key_frame_distance ||
        delta.so3().log().norm() > _options.key_frame_deg * M_PI / 180.0)
    {
        return true;
    }
    return false;
}

} // namespace algorithm_ns