#ifndef GRAPH_OPTIMIZER_INTERFACE_HPP_
#define GRAPH_OPTIMIZER_INTERFACE_HPP_

namespace glins
{

    class GraphOptimizerInterface
    {
    public:
        virtual ~GraphOptimizerInterface() {} // for executing child's destructor

        virtual void AddVertex() = 0;
        virtual void AddBinaryEdge() = 0;
        virtual void AddUnaryEdge() = 0;
        virtual void SetEdgeRobustKernel() = 0;
        virtual bool Optimize() = 0;

    }; // class GraphOptimizerInterface

} // namespace glins

#endif // GRAPH_OPTIMIZER_INTERFACE_HPP_