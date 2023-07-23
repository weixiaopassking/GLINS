#include "../cloud_registration_interface.hpp"

class ICPRegistration : public CloudRegistrationInterface
{
  public:
    ICPRegistration()
    {
    }
      void SetTarget() override;
    //  void SetSource() override;
    //  void GetPose() override;
      ~ICPRegistration(){};
};