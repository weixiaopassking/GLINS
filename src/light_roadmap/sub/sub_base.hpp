#ifndef _SUB_BASE_HPP
#define _SUB_BASE_HPP
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