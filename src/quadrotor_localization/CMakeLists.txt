add_library(pointcloud_handle lib/pointcloud_handle/pointcloud_handle.cpp) 

add_executable(test_node node/test_node.cpp)
add_test(test_node test_node)
target_link_libraries(test_node pointcloud_handle  time_utils ${thirdparty_libs} )

