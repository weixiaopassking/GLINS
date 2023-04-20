/*******************************
 *  @file     service.h
 *  @brief    调度器
 ********************************/

#include "service.hpp"
#include "pipelineBase.hpp"

// #include "./pipelinePerception/pipelinePerception.hpp"
#include "./pipelineLocalization/pipelineLocalization.hpp"

class service::impl
{
public:
    pipelineBase *getPipelinePerception();   // 感知pipeline: bev former、IPM
    pipelineBase *getPipelineLocalization(); // 定位pipeline: 融合lio or vio 与感知结果融合定位
    pipelineBase *getPipelineMapping();      // 建图pipeline: 地图管理与回环检测
    pipelineBase *getPipelinePlanning();     // 规控pipeline 混合Astar+mpc
    pipelineBase *getPipelineInteraction();  // 人机交互pipeline  车端rosqtUI+安卓端车位选择地图查看
private:
    // std::unique_ptr<pipelinePerception> _pipePerception_ptr;
    std::unique_ptr<pipelineLocalization> _pipeLocalization_ptr;
    // std::unique_ptr<pipelineMapping> _pipeMapping_ptr;
    // std::unique_ptr<pipelinePlanning> _pipeMapping_ptr;
};

service::service()
{
    _impl_ptr = std::make_unique<impl>();
}

service::~service()
{
}

void service::getPipeline(const std::string str)
{
    if(str.compare("定位")==0)
    {
    _impl_ptr->getPipelineLocalization();
    }

}

pipelineBase *service::impl::getPipelinePerception()
{

}

pipelineBase *service::impl::getPipelineLocalization()
{
        std::cout << "定位pipeline启动" << std::endl;
    _pipeLocalization_ptr = std::make_unique<pipelineLocalization>();
    return _pipeLocalization_ptr.get();
}

pipelineBase *service::impl::getPipelineMapping()
{
}

pipelineBase *service::impl::getPipelinePlanning()
{
}

pipelineBase *service::impl::getPipelineInteraction()
{
}
