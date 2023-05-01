# 使用ros melodic自带版本
find_package (Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
list(APPEND thirdpart_lib  Eigen3::Eigen)
