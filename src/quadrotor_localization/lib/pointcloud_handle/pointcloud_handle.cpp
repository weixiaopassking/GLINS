/**
*****************************************************************************
*  Copyright (C), 2023-2026,robotics gang
*  @file    pointcloud_handle.cpp
*  @brief  点云处理
*  @author  robotics gang
*  @date    2023/7/11
*  @version v0.1
*  @ref  github.com/gaoxiang12/slam_in_autonomous_driving
****************************************************************************
*/

#include "pointcloud_handle.hpp"

/**
 * @brief  点云处理初始化 重载1
 * @param source_path source点云 pcd的路径
 * @return
 * @note
 */
PointCloudHandle::PointCloudHandle(const std::string source_path)
{
    _cloud_source_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

    if (source_path.empty())
    {
        std::cout << "输入点云路径错误" << std::endl;
        exit(0);
    }
    pcl::io::loadPCDFile(source_path, *_cloud_source_ptr);

    if (_cloud_source_ptr->empty())
    {
        std::cout << "输入点云无效" << std::endl;
        exit(0);
    }
    else
    {
        std::cout << "点云数据规模为:" << _cloud_source_ptr->points.size() << std::endl;
    }
}

/**
 * @brief  点云处理初始化 重载2
 * @param source_path source点云 pcd的路径
 * @param target_path  target点云 pcd的路径
 * @return
 * @note
 */
PointCloudHandle::PointCloudHandle(const std::string source_path, const std::string target_path)
{
    _cloud_source_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _cloud_target_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

    if (source_path.empty() || target_path.empty())
    {
        std::cout << "输入点云路径错误" << std::endl;
    }
    pcl::io::loadPCDFile(source_path, *_cloud_source_ptr);
    pcl::io::loadPCDFile(target_path, *_cloud_target_ptr);

    if (_cloud_source_ptr->empty() || _cloud_target_ptr->empty())
    {
        std::cout << "输入点云无效" << std::endl;
    }
    else
    {
        std::cout << "source点云规模" << _cloud_source_ptr->points.size() << std::endl;
        std::cout << "target点云规模" << _cloud_target_ptr->points.size() << std::endl;
    }

    this->VoxelGridFilter(this->_cloud_source_ptr, 0.5);
    this->VoxelGridFilter(this->_cloud_target_ptr, 0.5);
    /*for debug*/
    std::cout << "滤波后source点云规模" << _cloud_source_ptr->points.size() << std::endl;
    std::cout << "滤波后target点云规模" << _cloud_target_ptr->points.size() << std::endl;
}

/**
 * @brief  点云处理初始化 重载3
 * @param cloud_source_ptr source点云指针
 * @return
 * @note
 */
PointCloudHandle::PointCloudHandle::PointCloudHandle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_ptr)
{
    _cloud_source_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _cloud_source_ptr = cloud_source_ptr;
}
/**
 * @brief  点云处理初始化 重载4
 * @param cloud_source_ptr source点云指针
 * @param cloud_target_ptr target点云指针
 * @return
 * @note
 */
PointCloudHandle::PointCloudHandle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_source_ptr,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_target_ptr)
{
    _cloud_source_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _cloud_target_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

    _cloud_source_ptr = cloud_source_ptr;
    _cloud_target_ptr = cloud_target_ptr;
}

/**
 * @brief  析构
 * @param none
 * @return
 * @note
 */
PointCloudHandle::~PointCloudHandle()
{
}

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

/**
 * @brief 点云k近邻
 * @param none
 * @return void
 * @note
 */
void PointCloudHandle::Knn()
{
    std::vector<std::pair<size_t, size_t>> matches; // source target


    std::vector<size_t> index(_cloud_target_ptr->points.size());
    std::for_each(index.begin(), index.end(),
                  [idx = 0](size_t &i) mutable { i = idx++; }); // mutable用于修改传入的队列

    matches.resize(index.size());

    std::for_each(index.begin(), index.end(), [&](auto idx) {
        matches[idx].second=idx;
         matches[idx].first= find_neighbour_index(_cloud_source_ptr, _cloud_target_ptr->points[idx]);
    });

}

/**
 * @brief 点云拟合平面(解析解)
 * @param points 观测点云
 * @param plane_coeffs 平面系数
 * @param eps 误差
 * @return bool
 * @note 静态成员函数
 */
bool PointCloudHandle::PlaneFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 4, 1> &plane_coeffs,
                                    const double eps)
{
    /*1--异常校验*/
    if (points.size() < 3)
    {
        return false;
    }
    /*2--设置A矩阵*/
    Eigen::MatrixXd A(points.size(), 4);
    for (int i = 0; i < points.size(); i++)
    {
        A.row(i).head<3>() = points[i].transpose();
        A.row(i)[3] = 1;
    }
    /*3--SVD分解*/
    Eigen::JacobiSVD svd(A, Eigen::ComputeThinV);
    plane_coeffs = svd.matrixV().col(3);

    /*4--误差校验*/
    for (int i = 0; i < points.size(); i++)
    {
        double error = plane_coeffs.head<3>().dot(points[i]) + plane_coeffs[3];
        if (error * error > eps)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 点云拟合直线(解析解)
 * @param points 观测点云
 * @param start_point 直线方程起点
 * @param direction 直线方程的方向向量
 * @return bool
 * @note 静态成员函数
 */
bool PointCloudHandle::LineFitting(std::vector<Eigen::Vector3d> &points, Eigen::Matrix<double, 3, 1> &start_point,
                                   Eigen::Matrix<double, 3, 1> &direction, const double eps)
{
    /*1--异常校验*/
    if (points.size() < 2)
        return false;
    /*2--计算输入点云的均匀值p*/
    start_point =
        std::accumulate(points.begin(), points.end(), Eigen::Matrix<double, 3, 1>::Zero().eval()) / points.size();
    /*3--计算xk-p=yk^T*/
    Eigen::MatrixXd Y(points.size(), 3);
    for (int i = 0; i < points.size(); i++)
    {
        Y.row(i) = (points[i] - start_point).transpose();
    }
    /*5--svd分解*/
    Eigen::JacobiSVD svd(Y, Eigen::ComputeFullV); // 雅可比分解
    direction = svd.matrixV().col(0);             // 取第一列即最大

    /*6--检查eps*/
    for (const auto &p : points)
    {
        if (direction.cross(p - start_point).squaredNorm() > eps)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 体素滤波器
 * @param cloud_ptr 点云指针
 * @param voxel_size 滤波器尺寸
 * @return void
 * @note 静态成员函数
 */
void PointCloudHandle::VoxelGridFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const float voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZI> filter_voxel;
    filter_voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    filter_voxel.setInputCloud(cloud_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    filter_voxel.filter(*cloud_filtered_ptr);
    cloud_ptr->swap(*cloud_filtered_ptr);
}

/**
 * @brief 打印调试
 * @param
 * @return void
 * @note friend  重载<<
 */
std::ostream &operator<<(std::ostream &o, const PointCloudHandle &s)
{
    if (s._cloud_source_ptr->empty())
    {
        o << "未输入原始点云" << std::endl;
    }
    if (s._cloud_target_ptr->empty())
    {
        o << "未输入目标点云" << std::endl;
    }

    return o;
}

/**
 * @brief 找最近邻 重载1
 * @param cloud_ptr
 * @param point
 * @return
 * @note 距离point最近的cloud_ptr对应的下标
 */
int PointCloudHandle::find_neighbour_index(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const pcl::PointXYZI& point)
{
    return std::min_element(cloud_ptr->points.begin(), cloud_ptr->points.end(),
                            [&point](const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2) -> bool {
                                return (pt1.getVector3fMap() - point.getVector3fMap()).squaredNorm() <
                                       (pt2.getVector3fMap() - point.getVector3fMap()).squaredNorm();
                            }) -
           cloud_ptr->points.begin();
}

/**
 * @brief 找最近邻 重载2
 * @param
 * @return
 * @note
 */
