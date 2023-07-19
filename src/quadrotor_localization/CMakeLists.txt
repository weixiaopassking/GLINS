add_library(cloud_handle_module  
lib/cloud_handle_module/cloud_io/cloud_io.cpp
lib/cloud_handle_module/cloud_fitting/cloud_fitting.cpp) 

add_executable(test_node node/test_node.cpp)
add_test(test_node test_node)
target_link_libraries(test_node cloud_handle_module  common ${thirdparty_libs} )

