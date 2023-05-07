find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 
list(APPEND thirdpart_lib ${OpenCV_LIBS})
 message("${BoldCyan}Opencv has been config${ColourReset}")