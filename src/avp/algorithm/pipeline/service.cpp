/*******************************
 *  @file     service.h
 *  @brief    调度器
 ********************************/

#include "service.hpp"

#include "./pipelineLocalization/pipelineLocalization.hpp"
#include "./pipelinePerception/pipelinePerception.hpp"
#include "pipelineBase.hpp"

class service::impl {
 public:
  pipelineBase *getPipelinePerception();
  pipelineBase *getPipelineLocalization();
  pipelineBase *getPipelineMapping();
  pipelineBase *getPipelinePlanning();
  pipelineBase *getPipelineInteraction();

 private:
  std::unique_ptr<pipelinePerception> _pipePerception_ptr;
  std::unique_ptr<pipelineLocalization> _pipeLocalization_ptr;
  // std::unique_ptr<pipelineMapping> _pipeMapping_ptr;
  // std::unique_ptr<pipelinePlanning> _pipeMapping_ptr;
  std::unique_ptr<pipelineTest> _pipeTest_ptr;
};

service::service() { _impl_ptr = std::make_unique<impl>(); }

service::~service() {}

void service::getPipeline(const std::string str) {
  if (str.compare("定位") == 0) {
    _impl_ptr->getPipelineLocalization();
  } else if (str.compare("感知") == 0) {
    _impl_ptr->getPipelinePerception();
  }
}

pipelineBase *service::impl::getPipelinePerception() {
  std::cout << "感知pipeline启动" << std::endl;
  _pipePerception_ptr = std::make_unique<pipelinePerception>();
  return _pipePerception_ptr.get();
}

pipelineBase *service::impl::getPipelineLocalization() {
  std::cout << "定位pipeline启动" << std::endl;
  _pipeLocalization_ptr = std::make_unique<pipelineLocalization>();
  return _pipeLocalization_ptr.get();
}

pipelineBase *service::impl::getPipelineMapping() {}

pipelineBase *service::impl::getPipelinePlanning() {}

pipelineBase *service::impl::getPipelineInteraction() {}
