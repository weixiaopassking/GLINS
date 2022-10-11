/*
 * @Description:自定义先验边
 * @Author: Robotics Gang
 * @Note: modified from Ren Qain 需要理解清楚
 * @Date: 2022-10-04
 */

#ifndef EDGE_SE3_PRIORXYZ_HPP_
#define EDGE_SE3_PRIORXYZ_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

/*GNSS 观测做先验边*/
namespace g2o
{
    /*观测维度 数据类型 链接顶点的类型*/
    class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3PriorXYZ() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}

        void computeError() override
        {
            const g2o::VertexSE3 *v1 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);

            Eigen::Vector3d estimate = v1->estimate().translation();
            _error = estimate - _measurement;
        }

        void setMeasurement(const Eigen::Vector3d &m) override
        {
            _measurement = m;
        }
        /*读盘*/
        bool read(std::istream &is) override
        {
            return true;
        }
        bool write(std::ostream &os) const override
        {
            return true;
        }
    };

} // namespace g2o

#endif