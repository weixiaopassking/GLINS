add_executable(test_node test_node.cpp)
target_include_directories(test_node   PUBLIC  ../sub)  #private 不会递归包含
target_link_libraries(test_node  ${project_name}.sub )

