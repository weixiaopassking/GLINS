#include "../cloud_registration_interface.hpp"




class ICPRegistration : public CloudRegistrationInterface
{
  public:
    ICPRegistration();

    void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud_ptr) override;
    void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) override;
    void SetGtTransform(const Sophus::SE3d& gt_transform) override;
     Sophus::SE3d GetResTransform() override;

    ~ICPRegistration();

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;
    Sophus::SE3d _gt_transform;
    bool has_gt_transform;

    Eigen::Vector3d _source_center_vec;
    Eigen::Vector3d _target_center_vec;
};