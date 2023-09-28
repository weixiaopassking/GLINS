#ifndef SUB_BASE_HPP_
#define SUB_BASE_HPP_
#include <deque>

namespace sub
{
class SubBase
{
  public:
    SubBase(){};

    // virtual void ParseData(...) = 0;
    // virtual void MsgCallback(...) = 0;
    virtual bool HasSubscribed() = 0;
    virtual ~SubBase(){};
};
} // namespace sub

#endif // SUB_BASE_HPP_