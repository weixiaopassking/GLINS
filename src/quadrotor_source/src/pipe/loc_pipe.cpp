#include "loc_pipe.hpp"

namespace pipe_ns
{

LocPipe::LocPipe()
{
    std::cout << "loc pipe 构造" << std::endl;
}
bool LocPipe::Run()
{
    std::cout << "loc pipe 运行" << std::endl;
    return true;
}
LocPipe::~LocPipe()
{
}

}; // namespace pipe_ns
