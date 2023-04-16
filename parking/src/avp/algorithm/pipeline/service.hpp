#include <string>
#include <memory>

class service
{
public:
    service();
    ~service();

    void getPipeline(const std::string str);

private:
    class impl;
    std::unique_ptr<impl> _impl_p;
};