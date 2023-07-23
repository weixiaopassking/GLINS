include_directories ("./lib/cloud_handle_module/cloud_filter")
include_directories ("./lib/cloud_handle_module/cloud_registration")  # 指定cuda头文件路径

add_library(cloud_handle_module  
lib/cloud_handle_module/cloud_io/cloud_io.cpp
lib/cloud_handle_module/cloud_fitting/cloud_fitting.cpp
lib/cloud_handle_module/cloud_viewer/cloud_viewer.cpp
lib/cloud_handle_module/cloud_fitting/cloud_fitting.cpp
lib/cloud_handle_module/cloud_filter/voxel_filter/voxel_filter.cpp
lib/cloud_handle_module/cloud_registration/icp_registration/icp_registration.cpp
) 


add_executable(test_node node/test_node.cpp)
target_link_libraries(test_node cloud_handle_module  common ${thirdparty_libs} )
add_test(test_node test_node)


