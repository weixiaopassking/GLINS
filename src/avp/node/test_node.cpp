#include <ros/ros.h>

#include <Eigen/Core>
#include <memory>

#include "eigenUsage.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_ndoe");
  ros::NodeHandle nh;

  std::unique_ptr<eigenDemo> demo_ptr = std::make_unique<eigenDemo>();
  return 0;
}
