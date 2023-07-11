#include "./basic_usage/pointcloud_handle/pointcloud_handle.hpp"
#include <bits/stdc++.h>
#include <memory>
using namespace std;

int main(int argc, char **argv)
{
    std::shared_ptr demo_ptr = std::make_shared<pointcloud_handle>("gang");
    return 0;
}