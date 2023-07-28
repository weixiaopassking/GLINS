set(loc_module_path ${project_path}/src/algorithm/loc_module)
include_directories(${project_path}/src/common)


add_library(${project_name}.algorithm.loc_module   

${loc_module_path}/cloud_handle/cloud_registration/icp_registration/icp_registration.cpp
) 

# add_library(${project_name}.algorithm.pnc_module   ) 