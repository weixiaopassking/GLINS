/**
 * @brief 点云k近邻
 * @param none
 * @return void
 * @note
 */
void PointCloudHandle::KnnInterface()
{
    std::vector<std::pair<size_t, size_t>> matches; // source target

    std::vector<size_t> index(_cloud_target_ptr->points.size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t &i) mutable { i = idx++; }); // mutable用于修改传入的队列

    matches.resize(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](auto idx) {
        matches[idx].second = idx;
        matches[idx].first = find_neighbour_index(_cloud_source_ptr, _cloud_target_ptr->points[idx]);
    });
}