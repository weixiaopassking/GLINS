add_library(${project_name}.sub   cloud_sub.cpp)
target_include_directories(${project_name}.sub   PUBLIC       ${project_path}/src/common)
 target_link_libraries(${project_name}.sub   ${project_name}.common   ${catkin_LIBRARIES}   ${thirdparty_LIBRARIES}    )


# add_executable(test_sub test_sub.cpp)


