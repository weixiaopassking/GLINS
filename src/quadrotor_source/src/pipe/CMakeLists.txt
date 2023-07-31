add_library(${project_name}.pipe.loc       loc_pipe.cpp)

target_include_directories(${project_name}.pipe.loc    PUBLIC    
${project_path}/src/algorithm/loc_module/lidar_odom
${project_path}/src/sub
${project_path}/src/pub
   )
 target_link_libraries(${project_name}.pipe.loc
 ${project_name}.algorithm.loc_module  
  ${project_name}.sub 
  ${project_name}.pub
  )



