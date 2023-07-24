include_directories ("../common")


add_executable(quick_test quick_test.cpp)
target_link_libraries(quick_test   common ${thirdparty_libs} )
add_test(quick_test COMMAND quick_test)


