#include "registration_interface.hpp"
#include <pcl/registration/ndt.h>

namespace module_ns
{

class NDT : public CloudRegistrationInterface
{

  public:
    NDT() ;

     void SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr) override;
     void SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr) override;
     void SetGtTransform(const data_ns::Mat4f &gt_transform) override;
     bool GetResTransform(data_ns::Mat4f &init_transform) override;

     ~NDT();

   private:
     pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
     pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;

     pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr _ndt_ptr;
};
} // namespace module_ns
