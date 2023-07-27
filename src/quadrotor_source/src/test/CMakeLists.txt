


# add_executable(test_cpp test_cpp.cpp)
# target_link_libraries(test_cpp     ${thirdparty_LIBRARIES} )
# add_test(test_cpp COMMAND test_cpp)

add_executable(test_module test_module.cpp)
target_link_libraries(test_module     ${thirdparty_LIBRARIES} )
add_test(test_module COMMAND test_module)
