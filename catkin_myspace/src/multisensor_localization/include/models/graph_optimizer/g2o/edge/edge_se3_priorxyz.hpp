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
    /*观测维度 输入的数据类型 链接顶点的类型*/
    class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
    {
    public:
         /*内存对齐*/
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
            Eigen::Vector3d v;
            is >> v(0) >> v(1) >> v(2);

            setMeasurement(Eigen::Vector3d(v));

            for (int i = 0; i < information().rows(); ++i)
                for (int j = i; j < information().cols(); ++j)
                {
                    is >> information()(i, j);
                    if (i != j)
                        information()(j, i) = information()(i, j);
                }
            return true;
        }
        bool write(std::ostream &os) const override
        {
            Eigen::Vector3d v = _measurement;
            os << v(0) << " " << v(1) << " " << v(2) << " ";
            for (int i = 0; i < information().rows(); ++i)
                for (int j = i; j < information().cols(); ++j)
                    os << " " << information()(i, j);
            return os.good();
        }
    };

} // namespace g2o

#endif