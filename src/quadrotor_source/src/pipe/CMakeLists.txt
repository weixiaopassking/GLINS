include_directories(${project_path}/src/algorithm/loc_module)
include_directories(${project_path}/src/common)

add_library(direct_lo direct_lo.cpp)
add_executable(test_pipe     test_pipe.cpp)
target_link_libraries(test_pipe direct_lo )
# target_link_libraries(direct_lo     ${thirdparty_LIBRARIES}  ${project_name}.algorithm.loc_module   )
# add_test(test_module COMMAND test_module)
