#include <iostream>

#include "../pipelineBase.hpp"

class pipelineLocalization : public pipelineBase {
 public:
  pipelineLocalization();
  ~pipelineLocalization();
  bool init() {}
  void run() {}
};
