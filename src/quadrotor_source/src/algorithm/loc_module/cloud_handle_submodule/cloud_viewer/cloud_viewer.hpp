#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudViewer
{

  public:
    CloudViewer() = delete;

    static void ViewerByOpencv(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const double image_resolution = 0.1,
                               const double z_lower = 0.2, const double z_upper = 2.5);
    static void ViewerByPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
    static void ViewerByRos();

    ~CloudViewer();
};
