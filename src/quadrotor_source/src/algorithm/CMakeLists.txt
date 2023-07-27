set(algorithm_part ${project_path}/src/algorithm)

add_library(${project_name}.algorithm.loc_module   

${algorithm_part}/loc_module/cloud_handle/cloud_registration/icp_registration/icp_registration.cpp
) 

# add_library(${project_name}.algorithm.pnc_module   ) 