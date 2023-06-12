#ifndef _ROS_SERVICE_ResetShooter_h
#define _ROS_SERVICE_ResetShooter_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tsuten_msgs
{

static const char RESETSHOOTER[] = "tsuten_msgs/ResetShooter";

  class ResetShooterRequest : public ros::Msg
  {
    public:

    ResetShooterRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return RESETSHOOTER; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetShooterResponse : public ros::Msg
  {
    public:

    ResetShooterResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return RESETSHOOTER; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetShooter {
    public:
    typedef ResetShooterRequest Request;
    typedef ResetShooterResponse Response;
  };

}
#endif
