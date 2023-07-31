

#loc_module
set(loc_module_path ${project_path}/src/algorithm/loc_module)

add_library(${project_name}.algorithm.loc_module   
${loc_module_path}/cloud_handle/cloud_registration/icp_registration/icp_registration.cpp
${loc_module_path}/lidar_odom/direct_lo.cpp

) 
target_include_directories(${project_name}.algorithm.loc_module  PUBLIC     ${project_path}/src/common)
 target_link_libraries(${project_name}.algorithm.loc_module      ${project_name}.common )
