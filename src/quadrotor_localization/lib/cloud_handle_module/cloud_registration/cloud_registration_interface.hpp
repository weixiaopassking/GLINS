#ifndef _CLOUD_REGISTRATION
#define _CLOUD_REGISTRATION

class CloudRegistrationInterface
{
  public:
    CloudRegistrationInterface() = delete;
    virtual void SetTarget() = 0;
    virtual void SetSource() = 0;
    virtual void GetPose();
    virtual ~CloudRegistrationInterface();
};

#endif //_CLOUD_REGISTRATION