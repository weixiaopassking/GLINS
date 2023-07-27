


add_executable(test_cpp test_cpp.cpp)
target_link_libraries(test_cpp   ${project_name}.common  ${thirdparty_LIBRARIES} )
add_test(test_cpp COMMAND test_cpp)


