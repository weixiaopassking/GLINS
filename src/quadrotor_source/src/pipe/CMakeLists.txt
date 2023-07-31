add_library(${project_name}.pipe.loc       loc_pipe.cpp)
target_include_directories(${project_name}.pipe.loc    PUBLIC       ${project_path}/src/sub)
 target_link_libraries(${project_name}.pipe.loc ${project_name}.sub )



