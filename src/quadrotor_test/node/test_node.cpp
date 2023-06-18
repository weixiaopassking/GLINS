#include "common/eigen_types.hpp"
#include "common/math_utils.hpp"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_stderrthreshold = google::INFO;              // INFO级以下不输出
    // FLAGS_colorlogtostderr = true;                     // 彩色日志
    // google::ParseCommandLineFlags(&argc, &argv, true); // 解析gflag

    std::cout << "this is test_node for testing" << std::endl;
    Vec3d v_body(1, 0, 0); // 车体自身速度
    Vec3d w_body(0, 0, 0); // 车体自身角速度
    Vec3d a_body(0, 0, 0); // 车体自身加速度
    Se3d pose;             // Twb
    const double dt = 1;   // 每次更新的时间
    while (1)
    {

        v_body += a_body * dt;
        Vec3d v_world = pose.so3() * v_body;
        Vec3d a_world = pose.so3() * a_body;

        pose.translation() += v_world * dt + 0.5 * a_world * dt * dt;
        pose.so3() = pose.so3() * So3::exp(w_body * dt);

        // 结果终端
        std::cout << "pose: "  << pose.translation().transpose() << std::endl;
        usleep(dt * 1e6); 
    }
}