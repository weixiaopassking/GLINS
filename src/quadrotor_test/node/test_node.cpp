#include "pose_reckon/pose_reckon.hpp"

#include <memory>

int main()
{
    std::cout << "this is just a  test" << std::endl;
    std::shared_ptr<pose_reckon> pose_reckon_ptr = std::make_shared<pose_reckon>();
}