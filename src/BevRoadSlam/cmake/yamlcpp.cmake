include(cmake/color.cmake)
find_package (yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})
list(APPEND thirdpart_lib ${YAML_CPP_LIBRARIES})
 message("${BoldCyan}Yamlcpp has been config${ColourReset}")