#ifndef _ROS_SERVICE_ResetOdometry_h
#define _ROS_SERVICE_ResetOdometry_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tsuten_msgs
{

static const char RESETODOMETRY[] = "tsuten_msgs/ResetOdometry";

  class ResetOdometryRequest : public ros::Msg
  {
    public:

    ResetOdometryRequest()
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

    virtual const char * getType() override { return RESETODOMETRY; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetOdometryResponse : public ros::Msg
  {
    public:

    ResetOdometryResponse()
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

    virtual const char * getType() override { return RESETODOMETRY; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetOdometry {
    public:
    typedef ResetOdometryRequest Request;
    typedef ResetOdometryResponse Response;
  };

}
#endif
