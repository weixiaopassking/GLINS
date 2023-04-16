/*
 * @Description:不做滤波
 * @Author: Robotic Gang
 *@Funciton:
 * @Note:Modified from Ren Qian
 * @Date: 2022-10-21
 */

// relevent
#include "../../../../include/models/graph_optimizer/g2o/g2o_optimizer.hpp"
// tools
#include "../../../../include/tools/color_terminal.hpp"
#include "../../../../include/tools/time_recorder.hpp"
// glog
#include <glog/logging.h>
// ros
#include <ros/ros.h>

namespace multisensor_localization
{

    /**
     * @brief 初始化优化算法
     * @note 通过工厂函数类生成固定搭配的优化算法，这样方便些
     * @todo
     **/
    G2oOptimizer::G2oOptimizer(const std::string &solver_type)
    {
        /*重置优化器指针*/
        graph_optimizer_ptr_.reset(new g2o::SparseOptimizer());
        /*为优化器配置算法*/
        g2o::OptimizationAlgorithmFactory *solver_factory = g2o::OptimizationAlgorithmFactory::instance();
        g2o::OptimizationAlgorithmProperty solver_property;
        g2o::OptimizationAlgorithm *solver = solver_factory->construct(solver_type, solver_property);
        graph_optimizer_ptr_->setAlgorithm(solver);
        /*检验优化器是否成功生成*/
        if (!graph_optimizer_ptr_->solver())
        {
            LOG(ERROR) << "[create g2o solver failed]" << std::endl;
            ROS_BREAK();
        }
        LOG(INFO) << "[create g2o solver success]" << std::endl
                  << solver_type << std::endl;
        /*初始化鲁棒核*/
        robust_kernel_factroy_ = g2o::RobustKernelFactory::instance();
    }

    /**
     * @brief 执行优化(核心代码)
     * @note
     * @todo
     **/
    bool G2oOptimizer::Optimize()
    {
        /*计数*/
        static int optimiz_cnt = 0;
        /*无边不优化*/
        if (graph_optimizer_ptr_->edges().size() < 1)
        {
            return false;
        }
        /*优化器设置*/
        graph_optimizer_ptr_->initializeOptimization();
        graph_optimizer_ptr_->computeInitialGuess();
        graph_optimizer_ptr_->computeActiveErrors();
        graph_optimizer_ptr_->setVerbose(false); //是否输出调试

        double cost_before_opt = graph_optimizer_ptr_->chi2();
        TimeRecorder opt_time_cost;
        int iterations = graph_optimizer_ptr_->optimize(max_iterations_num_); //最大迭代次数
        double cost_after_opt = graph_optimizer_ptr_->chi2();                 //代价值
        optimiz_cnt++;
        LOG(INFO) << "[g2o调试信息]" << std::endl
                  << "顶点数: " << graph_optimizer_ptr_->vertices().size() << std::endl
                  << "边数: " << graph_optimizer_ptr_->edges().size() << std::endl
                  << "耗时ms:  " << opt_time_cost.Result() << std::endl
                  << "误差变化:  " << cost_before_opt << "-----" << cost_after_opt << std::endl
                  << "迭代次数: " << iterations << " / " << max_iterations_num_ << std::endl
                  << "优化次数: " << optimiz_cnt << std::endl;

        // ColorTerminal::ColorInfo("g2o迭代记录");

        return true;
    }

    /**
     * @brief 添加鲁棒核
     * @note 鲁棒核主要用于避免异常点带来的误差
     * @todo
     **/
    void G2oOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size)
    {
        robust_kernel_name_ = robust_kernel_name;
        robust_kernel_size_ = robust_kernel_size;
        need_robust_kernel_ = true;
    }
    /**
     * @brief 添加顶点
     * @note 通常第一个顶点为先验，固定不优化
     * @todo
     **/
    void G2oOptimizer::AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix)
    {
        g2o::VertexSE3 *vertex(new g2o::VertexSE3());
        vertex->setId(graph_optimizer_ptr_->vertices().size());
        vertex->setEstimate(pose);
        if (need_fix)
        {
            vertex->setFixed(true);
        }
        graph_optimizer_ptr_->addVertex(vertex);
    }
    /**
     * @brief 添加边
     * @note
     * @todo
     **/
    void G2oOptimizer::AddSe3Edge(int vertex_index1,
                                  int vertex_index2,
                                  const Eigen::Isometry3d &relative_pose,
                                  const Eigen::VectorXd noise)
    {
        /*计算信息矩阵 */
        Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
        for (int i = 0; i < noise.rows(); i++)
        {
            information_matrix(i, i) /= noise(i);
        }

        g2o::VertexSE3 *v1 = dynamic_cast<g2o::VertexSE3 *>(graph_optimizer_ptr_->vertex(vertex_index1));
        g2o::VertexSE3 *v2 = dynamic_cast<g2o::VertexSE3 *>(graph_optimizer_ptr_->vertex(vertex_index2));
        g2o::EdgeSE3 *edge(new g2o::EdgeSE3());

        edge->setMeasurement(relative_pose);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v1;
        edge->vertices()[1] = v2;
        graph_optimizer_ptr_->addEdge(edge);

        if (need_robust_kernel_)
        {
            AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
        }
    }

    /**
     * @brief 添加鲁棒核
     * @note
     * @todo
     **/
    void G2oOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge *edge,
                                       const std::string &kernel_type,
                                       double kernel_size)
    {
        /*鲁棒核类型为None:不添加鲁棒核*/
        if (kernel_type == "None")
        {
            return;
        }
        /*否则根据类型添加鲁棒核*/
        g2o::RobustKernel *kernel = robust_kernel_factroy_->construct(kernel_type);
        if (kernel == nullptr)
        {
            //   ColorTerminal::ColorInfo("无对应的鲁棒核");
        }
        kernel->setDelta(kernel_size);
        edge->setRobustKernel(kernel);
    }

    /**
     * @brief 添加先验边
     * @note
     * @todo
     **/
    void G2oOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                          const Eigen::Vector3d &xyz,
                                          const Eigen::VectorXd noise)
    {
        /*计算信息矩阵*/
        Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
        for (int i = 0; i < noise.rows(); i++)
        {
            information_matrix(i, i) /= noise(i);
        }
        /*创建先验边*/
        g2o::VertexSE3 *v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_optimizer_ptr_->vertex(se3_vertex_index));
        g2o::EdgeSE3PriorXYZ *edge(new g2o::EdgeSE3PriorXYZ());
        edge->setMeasurement(xyz);
        edge->setInformation(information_matrix);
        edge->vertices()[0] = v_se3;
        graph_optimizer_ptr_->addEdge(edge);
    }

    /**
     * @brief 输出节点个数
     * @note
     * @todo
     **/
    int G2oOptimizer::GetNodeNum()
    {
        return graph_optimizer_ptr_->vertices().size();
    }
    /**
     * @brief 得到优化后的位姿
     * @note 这里是全局优化
     * @todo
     **/
    bool G2oOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f> &optimized_pose)
    {
        optimized_pose.clear();
        int vertex_num = graph_optimizer_ptr_->vertices().size();
        for (int i = 0; i < vertex_num; i++)
        {
            g2o::VertexSE3 *v = dynamic_cast<g2o::VertexSE3 *>(graph_optimizer_ptr_->vertex(i));
            Eigen::Isometry3d pose = v->estimate();
            optimized_pose.push_back(pose.matrix().cast<float>());
        }
        return true;
    }

} // namespace multisensor_localization

// gn_var_cholmod       CHOLMOD    Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)
// gn_fix3_2_cholmod    CHOLMOD    Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)
// gn_fix6_3_cholmod    CHOLMOD    Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)
// gn_fix7_3_cholmod    CHOLMOD    Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)
// lm_var_cholmod       CHOLMOD    Levenberg: Cholesky solver using CHOLMOD (variable blocksize)
// lm_fix3_2_cholmod    CHOLMOD    Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)
// lm_fix6_3_cholmod    CHOLMOD    Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)
// lm_fix7_3_cholmod    CHOLMOD    Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)
// dl_var_cholmod       CHOLMOD    Dogleg: Cholesky solver using CHOLMOD (variable blocksize)
// gn_var_csparse       CSparse    Gauss-Newton: Cholesky solver using CSparse (variable blocksize)
// gn_fix3_2_csparse    CSparse    Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)
// gn_fix6_3_csparse    CSparse    Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)
// gn_fix7_3_csparse    CSparse    Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)
// lm_var_csparse       CSparse    Levenberg: Cholesky solver using CSparse (variable blocksize)
// lm_fix3_2_csparse    CSparse    Levenberg: Cholesky solver using CSparse (fixed blocksize)
// lm_fix6_3_csparse    CSparse    Levenberg: Cholesky solver using CSparse (fixed blocksize)
// lm_fix7_3_csparse    CSparse    Levenberg: Cholesky solver using CSparse (fixed blocksize)
// dl_var_csparse       CSparse    Dogleg: Cholesky solver using CSparse (variable blocksize)
// gn_pcg               PCG        Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (variable blocksize)
// gn_pcg3_2            PCG        Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
// gn_pcg6_3            PCG        Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
// gn_pcg7_3            PCG        Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
// lm_pcg               PCG        Levenberg: PCG solver using block-Jacobi pre-conditioner (variable blocksize)
// lm_pcg3_2            PCG        Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
// lm_pcg6_3            PCG        Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
// lm_pcg7_3            PCG        Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)
