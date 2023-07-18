add_library(cloud_handle_module  lib/cloud_handle_module/cloud_io/cloud_io.cpp) 

add_executable(test_node node/test_node.cpp)
add_test(test_node test_node)
target_link_libraries(test_node cloud_handle_module  time_utils ${thirdparty_libs} )

