# sudo apt install libeigen3-dev
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# list(APPEND thirdpart_lib  Eigen3::Eigen)
