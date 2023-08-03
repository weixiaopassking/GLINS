#include "cloud_registration_interface.hpp"
#include <pcl/registration/ndt.h>

namespace module_ns
{

class NDTRegistration : public CloudRegistrationInterface
{
    struct Options
    {
        float res = 1.0;
        float step_size = 0.1;
        float trans_eps = 0.01;
        int max_iter = 30;
    };

  public:
    NDTRegistration();

    void SetSourceCloud(const data_ns::CloudData::CLOUD_PTR &source_cloud_ptr) override;
    void SetTargetCloud(const data_ns::CloudData::CLOUD_PTR &target_cloud_ptr) override;
    void SetGtTransform(const data_ns::Mat4f &gt_transform) override;
    bool GetResTransform(data_ns::Mat4f &init_transform) override;

    ~NDTRegistration();

  private:
    // pcl::PointCloud<pcl::PointXYZI>::Ptr _source_cloud_ptr;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr _target_cloud_ptr;
    Options _option;
    pcl::NormalDistributionsTransform<data_ns::CloudData::POINT, data_ns::CloudData::POINT>::Ptr _ndt_registration_ptr;
};
} // namespace module_ns
