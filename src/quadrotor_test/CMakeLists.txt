
#自建库文件
add_library(pose_reckon algorithm_example/pose_reckon/pose_reckon.cpp)
add_library(sophus_use library_example/sophus/sophus_use.cpp)


add_executable(test_node node/test_node.cpp)
target_link_libraries(test_node
 ${thirdpartlib}
  pose_reckon
  sophus_use
  )

include_directories(algorithm_example)
include_directories(library_example)