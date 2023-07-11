add_library(demo basic_usage/pointcloud_handle/pointcloud_handle.cpp) 

add_executable(test_node test_node.cpp)
target_link_libraries(test_node demo ${thirdparty_libs} )
