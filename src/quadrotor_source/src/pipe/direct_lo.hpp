#include "cloud_handle/cloud_registration/cloud_registration_interface.hpp"
#include "cloud_handle/cloud_registration/icp_registration/icp_registration.hpp"
#include <deque>
#include <pcl/common/transforms.h>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

namespace pipe_ns
{

class DirectLo
{
  public:
    enum RegistrationMethods
    {
        ICP,
        NDT,
        INC_NDT
    };
    struct Options
    {
        double key_frame_distance = 0.5;   // 关键帧距离
        double key_frame_deg = 30;          // 关键帧角度
        int local_map_key_frames_num = 30; //
        RegistrationMethods registration_method = RegistrationMethods::ICP;
    };

    DirectLo();
    bool Update(pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr, Sophus::SE3d &pose);
    bool SaveMap();
    ~DirectLo();

  private:
    bool IsKeyFrame(const Sophus::SE3d &current_pose);

    Options _optins;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> _local_map_scans_que; // 激光点云队列
    pcl::PointCloud<pcl::PointXYZI>::Ptr _local_map_ptr;                   // 局部地图
    Sophus::SE3d _last_key_fram_pose;                                      // 上一关键帧位姿

    std::vector<Sophus::SE3d> _estimated_poses_vec; // 历史位姿势

    std::shared_ptr<algorithm_ns::CloudRegistrationInterface> _cloud_registration_ptr;
};

} // namespace pipe_ns