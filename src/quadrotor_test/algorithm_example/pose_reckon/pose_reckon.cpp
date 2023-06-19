#include "pose_reckon.hpp"

pose_reckon::pose_reckon()
{
    this->init();
    while (1)
    {
        this->run();
        usleep(dt * 1e6);
    }
}

void pose_reckon ::init()
{
    v_body << 1, 0, 0;
    w_body << 1, 2, 3;
    a_body << 0, 0, 0;
    // ego_pose; //todo init
    dt = 1.0;
}
void pose_reckon::run()
{

    v_body += a_body * dt;
    Vec3d v_world = ego_pose.so3() * v_body;
    Vec3d a_world = ego_pose.so3() * a_body;

    ego_pose.translation() += v_world * dt + 0.5 * a_world * dt * dt;
    ego_pose.so3() = ego_pose.so3() * So3::exp(w_body * dt);

    std::cout << "pose: " << std::endl << ego_pose.matrix() << std::endl;
}
