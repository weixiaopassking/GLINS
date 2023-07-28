include_directories(${project_path}/src/algorithm/loc_module)
include_directories(${project_path}/src/common)

#测试第三方库使用
# add_executable(test_libs test_libs.cpp)
# target_link_libraries(test_libs     ${thirdparty_LIBRARIES} )
# add_test(test_libs COMMAND test_libs)


#测试cpp特性
# add_executable(test_cpp test_cpp.cpp)
# target_link_libraries(test_cpp     ${thirdparty_LIBRARIES} )
# add_test(test_cpp COMMAND test_cpp)

#测试算法模块
add_executable(test_module test_module.cpp)
target_link_libraries(test_module     ${thirdparty_LIBRARIES}  ${project_name}.algorithm.loc_module   )
add_test(test_module COMMAND test_module)
