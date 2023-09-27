#ifndef SUB_BASE_HPP_
#define SUB_BASE_HPP_
#include <deque>

class SubBase
{
  public:
    SubBase(){};

    // virtual void ParseData(...) = 0;
    // virtual void MsgCallback(...) = 0;
    virtual bool HasSubscribed() = 0;
    virtual ~SubBase(){};
};

#endif // SUB_BASE_HPP_