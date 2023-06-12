#ifndef _ROS_jsk_rviz_plugins_Pictogram_h
#define _ROS_jsk_rviz_plugins_Pictogram_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"

namespace jsk_rviz_plugins
{

  class Pictogram : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef uint8_t _action_type;
      _action_type action;
      typedef uint8_t _mode_type;
      _mode_type mode;
      typedef const char* _character_type;
      _character_type character;
      typedef double _size_type;
      _size_type size;
      typedef double _ttl_type;
      _ttl_type ttl;
      typedef double _speed_type;
      _speed_type speed;
      typedef std_msgs::ColorRGBA _color_type;
      _color_type color;
      enum { ADD = 0 };
      enum { DELETE = 1 };
      enum { ROTATE_Z = 2 };
      enum { ROTATE_Y = 3 };
      enum { ROTATE_X = 4 };
      enum { JUMP = 5 };
      enum { JUMP_ONCE = 6 };
      enum { PICTOGRAM_MODE = 0 };
      enum { STRING_MODE = 1 };

    Pictogram():
      header(),
      pose(),
      action(0),
      mode(0),
      character(""),
      size(0),
      ttl(0),
      speed(0),
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->action >> (8 * 0)) & 0xFF;
      offset += sizeof(this->action);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      uint32_t length_character = strlen(this->character);
      varToArr(outbuffer + offset, length_character);
      offset += 4;
      memcpy(outbuffer + offset, this->character, length_character);
      offset += length_character;
      union {
        double real;
        uint64_t base;
      } u_size;
      u_size.real = this->size;
      *(outbuffer + offset + 0) = (u_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_size.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_size.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_size.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_size.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_size.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->size);
      union {
        double real;
        uint64_t base;
      } u_ttl;
      u_ttl.real = this->ttl;
      *(outbuffer + offset + 0) = (u_ttl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ttl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ttl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ttl.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ttl.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ttl.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ttl.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ttl.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ttl);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed);
      offset += this->color.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      this->action =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->action);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
      uint32_t length_character;
      arrToVar(length_character, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_character; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_character-1]=0;
      this->character = (char *)(inbuffer + offset-1);
      offset += length_character;
      union {
        double real;
        uint64_t base;
      } u_size;
      u_size.base = 0;
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_size.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->size = u_size.real;
      offset += sizeof(this->size);
      union {
        double real;
        uint64_t base;
      } u_ttl;
      u_ttl.base = 0;
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ttl.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ttl = u_ttl.real;
      offset += sizeof(this->ttl);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      offset += this->color.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "jsk_rviz_plugins/Pictogram"; };
    virtual const char * getMD5() override { return "29667e5652a8cfdc9c87d2ed97aa7bbc"; };

  };

}
#endif
