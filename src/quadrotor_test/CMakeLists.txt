
#自建库文件
add_library(pointcloud_handle basic_usage/pointcloud_handle/pointcloud_handle.cpp ) 



add_executable(test_node test_node/test_node.cpp)
target_link_libraries(test_node
 ${thirdpartlib}
 pointcloud_handle
  )
include_directories(basic_usage)
include_directories(lib_usage)