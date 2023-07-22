#include "../cloud_registration_interface.hpp"

class ICP : public CloudRegistrationInterface
{
  public:
    virtual void SetTarget() override;
    virtual void SetSource() override;
    virtual void GetPose() override;

};