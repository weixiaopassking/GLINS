#ifndef _PLANNING_MSG_HPP
#define _PLANNING_MSG_HPP

namespace PlanningMsgNS
{

// 路径搜索的节点 单向链表
template <typename T> struct Node
{

    T x;
    T y;
    T z;
    Node *next;
    Node():next(nullptr)
    {
    }
    Node(T x_, T y_, T z_) : (x)x, (y)y, z(z), next(nullptr)
    {
    }
};
} // namespace PlanningMsgNS

#endif //_PLANNING_MSG_HPP