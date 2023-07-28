#include "./direct_lo.hpp"

namespace pipe_ns
{
DirectLo::DirectLo()
{
    /*1. 选择配准方法*/
    if (_optins.registration_method == RegistrationMethods::ICP)
    {
        _cloud_registration_ptr = std::make_shared<algorithm_ns::ICPRegistration>();
    }
    else
    {
        common_ns::ErrorAssert("无对应的配置方法", __FILE__, __FUNCTION__, __LINE__);
    }
    /*2 重置变量*/
    _local_map_ptr.reset();
    _local_map_scans_que.clear();
    _estimated_poses_vec.clear();
}

bool DirectLo::Update(pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr, Sophus::SE3d &current_pose)
{
    /*1--处理第一帧*/
    if (_local_map_ptr == nullptr)
    {
        current_pose = Sophus::SE3d(); // 初始化第一帧地图和scan都是地图源点
        _last_key_fram_pose = current_pose;
        (*_local_map_ptr) += (*scan_ptr);
        _cloud_registration_ptr->SetTargetCloud(_local_map_ptr);
        return true;
    }
    /*2--处理之后帧*/
    _cloud_registration_ptr->SetSourceCloud(scan_ptr);
    _cloud_registration_ptr->GetResTransform(current_pose); // scan to local_map
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_world(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*scan_ptr, *scan_world, current_pose.matrix().cast<float>()); // scan转至世界系

    /*3--p判断是否为关键帧*/
    if (IsKeyFrame(current_pose) == true)
    {
        _last_key_fram_pose = current_pose;
    }

    
}

bool DirectLo::SaveMap()
{
    return true;
}
DirectLo::~DirectLo()
{
}

bool DirectLo::IsKeyFrame(const Sophus::SE3d &current_pose)
{
    Sophus::SE3d dtlta = _last_key_fram_pose.inverse() * current_pose;
    if (delta.translation().norm() > _optins.key_frame_distance ||
        delta.so3().log().norm() > _option.key_frame_deg * M_PI / 180.0)
    {
        return false;
    }
    return true;
}

} // namespace pipe_ns