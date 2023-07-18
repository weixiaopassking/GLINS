/**
 * @brief pcd转换bev image#include <numeric>
 * @param image_resolution 分辨率默认0.05
 * @return void
 * @note
 *  坐标系
 * ——x(cols)
 * |
 * y(rows)
 */
void PointCloudHandle::GenerateBevImage(const double image_resolution, const double z_upper, const double z_lower)
{
    /*1--计算点云边界*/
    auto minmax_x = std::minmax_element(_cloud_source_ptr->points.begin(), _cloud_source_ptr->points.end(),
                                        [](const pcl::PointXYZI &p1, const pcl::PointXYZI &p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(_cloud_source_ptr->points.begin(), _cloud_source_ptr->points.end(),
                                        [](const pcl::PointXYZI &p1, const pcl::PointXYZI &p2) { return p1.y < p2.y; });

    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;
    const double inv_r = 1.0 / image_resolution;

    /*2--计算图像数据*/
    const int image_rows = int((max_y - min_y) * inv_r);
    const int image_cols = int((max_x - min_x) * inv_r);

    float image_rows_center = image_rows * 0.5;
    float image_cols_center = image_cols * 0.5;

    float x_center = (min_x + max_x) * 0.5;
    float y_center = (min_y + max_y) * 0.5;

    /*3--生成图像*/
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto &pt : _cloud_source_ptr->points)
    {
        int x = int((pt.x - x_center) * inv_r + image_cols_center);
        int y = int((pt.y - y_center) * inv_r + image_rows_center);
        if (x < 0 || y < 0 || x >= image_cols || y >= image_rows || pt.z < z_lower || pt.z > z_upper)
        {
            continue;
        }

        image.at<cv::Vec3b>(y, x) = cv::Vec3b(227, 143, 79);
    }
    cv::imshow("Display2d", image);
    cv::waitKey(0);
    cv::imwrite("./bev.png", image);
}

/**
 * @brief pcl库3D可视化
 * @param
 * @return void
 * @note
 */
void PointCloudHandle::Display()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Display3d"));
    viewer->setBackgroundColor(0, 0, 0); // 黑色
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handle(_cloud_source_ptr,
                                                                                  "z"); // 使用高度来着色
    viewer->addPointCloud<pcl::PointXYZI>(_cloud_source_ptr, handle);
    viewer->spin(); // 自旋
}