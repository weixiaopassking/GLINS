add_library(pose_reckon algorithm_example/pose_reckon/pose_reckon.cpp)
add_executable(test_node node/test_node.cpp)
target_link_libraries(test_node ${thirdpartlib} pose_reckon)

include_directories(algorithm_example)