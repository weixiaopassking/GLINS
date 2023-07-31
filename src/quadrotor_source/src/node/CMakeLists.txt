add_executable(test_node test_node.cpp)
target_include_directories(test_node   PUBLIC     ${project_path}/src/pipe)  
target_link_libraries(test_node  ${project_name}.pipe.loc)

