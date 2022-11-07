/*
 * @Description:
 * @Author: Robotics Gang
 * @Note: modified from Ren Qain 需要理解清楚
 * @Date: 2022-10-04
 */

#ifndef GRAPH_OPTIMIZER_INTERFACE_HPP_
#define GRAPH_OPTIMIZER_INTERFACE_HPP_

#include <Eigen/Dense>
#include <deque>


namespace multisensor_localization
{

    class GraphOptimizerInterface
    {
    public:
        virtual ~GraphOptimizerInterface() {}
        /*优化动作*/
        virtual bool Optimize() = 0;
        // /*输入输出数据*/
        virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f> &Optimized_pose) = 0;
        virtual int GetNodeNum() = 0;
        // /*添加鲁棒核、节点、边、*/
        virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
         virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;
        virtual void AddSe3Edge(int vertex_index1,
                                int vertex_index2,
                                const Eigen::Isometry3d &relative_pose,
                                const Eigen::VectorXd noise) = 0;
        virtual void AddSe3PriorXYZEdge(int se3_vertex_index,
                                        const Eigen::Vector3d &xyz,
                                        const Eigen::VectorXd noise)=0;

        /*设置优化参数*/
        void SetMaxIterationsNum(int max_iterations_num);

        protected://! 可继承但不可访问
        int max_iterations_num_=512;
    };

} // namespace multisensor_localization

#endif