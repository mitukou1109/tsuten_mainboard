#ifndef _ROS_SERVICE_ShootBottle_h
#define _ROS_SERVICE_ShootBottle_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tsuten_msgs
{

static const char SHOOTBOTTLE[] = "tsuten_msgs/ShootBottle";

  class ShootBottleRequest : public ros::Msg
  {
    public:

    ShootBottleRequest()
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

    virtual const char * getType() override { return SHOOTBOTTLE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ShootBottleResponse : public ros::Msg
  {
    public:

    ShootBottleResponse()
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

    virtual const char * getType() override { return SHOOTBOTTLE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ShootBottle {
    public:
    typedef ShootBottleRequest Request;
    typedef ShootBottleResponse Response;
  };

}
#endif
