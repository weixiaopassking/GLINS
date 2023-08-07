#ifndef _GRAPH_OPTIM_INTERFACE_HPP
#define _GRAPH_OPTIM_INTERFACE_HPP
/*
 * @Description: graph_interface for gtsam g2o
 * @Function:
 * @Author: wengang.niu
 * @Version : v1.0
 * @Date: 2023-08-06
 */

namespace module_ns
{
class GraphOptimInterface
{
  public:
    GraphOptimInterface() = default;

    virtual bool GetOptimizedPose() = 0;

    virtual void AddVertex()=0;
    virtual void AddEdge()=0;
    virtual void AddPriorEdge()=0;


    virtual ~GraphOptimInterface() = default;
}; // class GraphOptimInterface
} // namespace module_ns

#endif //_GRAPH_OPTIM_INTERFACE_HPP