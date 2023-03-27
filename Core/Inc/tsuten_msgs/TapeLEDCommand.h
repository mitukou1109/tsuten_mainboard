#ifndef _ROS_tsuten_msgs_TapeLEDCommand_h
#define _ROS_tsuten_msgs_TapeLEDCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/ColorRGBA.h"

namespace tsuten_msgs
{

  class TapeLEDCommand : public ros::Msg
  {
    public:
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;
      typedef bool _blink_type;
      _blink_type blink;

    TapeLEDCommand():
      color(),
      blink(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->color.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_blink;
      u_blink.real = this->blink;
      *(outbuffer + offset + 0) = (u_blink.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blink);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->color.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_blink;
      u_blink.base = 0;
      u_blink.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blink = u_blink.real;
      offset += sizeof(this->blink);
     return offset;
    }

    virtual const char * getType() override { return "tsuten_msgs/TapeLEDCommand"; };
    virtual const char * getMD5() override { return "9bb129c94deb12e7bfb8e2cc955826c8"; };

  };

}
#endif
