#include "../cloud_registration_interface.hpp"

class ICP : public CloudRegistrationInterface
{
  public:
    ICP(){}
      void SetTarget() override;
    //  void SetSource() override;
    //  void GetPose() override;
    ~ICP(){};
};