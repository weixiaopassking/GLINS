add_executable(vis_node vis_node.cpp)
# target_link_libraries(vis_node cloud_handle_module  common ${thirdparty_libs} )
target_link_libraries(vis_node ${catkin_LIBRARIES}  ${thirdparty_LIBRARIES} )