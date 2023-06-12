#ifndef _ROS_tsuten_msgs_SensorStates_h
#define _ROS_tsuten_msgs_SensorStates_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tsuten_msgs
{

  class SensorStates : public ros::Msg
  {
    public:
      typedef bool _bumper_l_type;
      _bumper_l_type bumper_l;
      typedef bool _bumper_r_type;
      _bumper_r_type bumper_r;

    SensorStates():
      bumper_l(0),
      bumper_r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_bumper_l;
      u_bumper_l.real = this->bumper_l;
      *(outbuffer + offset + 0) = (u_bumper_l.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bumper_l);
      union {
        bool real;
        uint8_t base;
      } u_bumper_r;
      u_bumper_r.real = this->bumper_r;
      *(outbuffer + offset + 0) = (u_bumper_r.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->bumper_r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_bumper_l;
      u_bumper_l.base = 0;
      u_bumper_l.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bumper_l = u_bumper_l.real;
      offset += sizeof(this->bumper_l);
      union {
        bool real;
        uint8_t base;
      } u_bumper_r;
      u_bumper_r.base = 0;
      u_bumper_r.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->bumper_r = u_bumper_r.real;
      offset += sizeof(this->bumper_r);
     return offset;
    }

    virtual const char * getType() override { return "tsuten_msgs/SensorStates"; };
    virtual const char * getMD5() override { return "3e34d3064d00b3b4db1658adf7cd00d1"; };

  };

}
#endif
