#include "pose_reckon/pose_reckon.hpp"
#include "sophus/sophus_use.hpp"

#include <memory>

int main()
{
    std::cout << "this is just a  test" << std::endl;
    // std::shared_ptr<pose_reckon> pose_reckon_ptr = std::make_shared<pose_reckon>();
    std::shared_ptr<sophus_use> pose_reckon_ptr = std::make_shared<sophus_use>();
}