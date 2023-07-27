// /**
//  * @brief 找最近邻 重载1
//  * @param cloud_ptr
//  * @param point
//  * @return
//  * @note 距离point最近的cloud_ptr对应的下标
//  */
// int PointCloudHandle::find_neighbour_index(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const pcl::PointXYZI&
// point)
// {
//     return std::min_element(cloud_ptr->points.begin(), cloud_ptr->points.end(),
//                             [&point](const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2) -> bool {
//                                 return (pt1.getVector3fMap() - point.getVector3fMap()).squaredNorm() <
//                                        (pt2.getVector3fMap() - point.getVector3fMap()).squaredNorm();
//                             }) -
//            cloud_ptr->points.begin();
// }
