add_library(template_module
lib/template_module/template_submodule/submodule.cpp) 

add_executable(template_node node/template_node.cpp)
target_link_libraries(template_node template_module   common ${thirdparty_libs} )

