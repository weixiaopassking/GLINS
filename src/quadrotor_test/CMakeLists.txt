add_library(pointcloud_handle basic_usage/pointcloud_handle/pointcloud_handle.cpp) 

add_executable(test_node node/test_node.cpp)
target_link_libraries(test_node pointcloud_handle ${thirdparty_libs} )
