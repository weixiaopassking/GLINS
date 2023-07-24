#include "../cloud_registration_interface.hpp"
#include <execution>

class ICPRegistration : public CloudRegistrationInterface
{

  public:
    struct Options
    {
        const int max_iteration = 20;
        const int min_nn_numbers = 10;

        const double max_point2point_distance = 1.0;
        const double max_point2line_distance = 0.5;
        const double max_point2plane_distance = 0.05;

        const double epsilon = 1e-2;
    };
    ICPRegistration();

    void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr) override;
    void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) override;
    void SetGtTransform(const Sophus::SE3d &gt_transform) override;
   void  GetResTransform(Sophus::SE3d &init_transform) override;

    ~ICPRegistration();

  private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;
    Sophus::SE3d _gt_transform;
    bool has_gt_transform;

    Eigen::Vector3d _source_center_vec;
    Eigen::Vector3d _target_center_vec;
    Options options_;
};