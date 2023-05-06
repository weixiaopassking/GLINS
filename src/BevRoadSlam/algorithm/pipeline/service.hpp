/*******************************
 *  @file     service.h
 *  @brief    调度器
 ********************************/
#ifndef _SERVICE_HPP
#define _SERVICE_HPP

#include <memory>
#include <string>

class service {
 public:
  service();
  ~service();

  void getPipeline(const std::string str);

 private:
  class impl;
  std::unique_ptr<impl> _impl_ptr;
};

#endif