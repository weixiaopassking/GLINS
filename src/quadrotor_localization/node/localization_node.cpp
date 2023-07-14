#include "../pipeline/localization_pipeline.hpp"
#include <iostream>
#include <memory>

int main(int argc, char **argv)
{

    std::cout << "localization starting" << std::endl;
    std::unique_ptr<LocaizationPipeline> pipe_loc_ptr = std::make_unique<LocaizationPipeline>();
}