#ifndef _CLOUD_REGISTRATION_HPP
#define _CLOUD_REGISTRATION_HPP

class CloudRegistrationInterface
{
  public:
    CloudRegistrationInterface(){};
    virtual void SetTarget() = 0;
    // virtual void SetSource() = 0;
    // virtual void GetPose()=0;
    virtual ~CloudRegistrationInterface(){};
};

#endif //_CLOUD_REGISTRATION
