
#include <pcl/registration/ndt.h>
#include "cloud_registration_interface.hpp"
#include <execution>


namespace algorithm_ns
{

class NDTRegistration : public CloudRegistrationInterface
{

  public:
    NDTRegistration()=default;

    void SetSourceCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud_ptr) override;
    void SetTargetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud_ptr) override;
    void SetGtTransform(const Sophus::SE3d &gt_transform) override;
    bool GetResTransform(Sophus::SE3d &init_transform) override;

    ~NDTRegistration();

  private:


    pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI > _ndt_pcl;
};
} // namespace algorithm_ns
