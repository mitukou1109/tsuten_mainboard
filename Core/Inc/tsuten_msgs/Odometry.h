#ifndef _ROS_tsuten_msgs_Odometry_h
#define _ROS_tsuten_msgs_Odometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace tsuten_msgs
{

  class Odometry : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef int16_t _x_type;
      _x_type x;
      typedef int16_t _y_type;
      _y_type y;
      typedef int16_t _theta_type;
      _theta_type theta;
      typedef int16_t _v_x_type;
      _v_x_type v_x;
      typedef int16_t _v_y_type;
      _v_y_type v_y;
      typedef int16_t _v_theta_type;
      _v_theta_type v_theta;

    Odometry():
      stamp(),
      x(0),
      y(0),
      theta(0),
      v_x(0),
      v_y(0),
      v_theta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x);
      union {
        int16_t real;
        uint16_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y);
      union {
        int16_t real;
        uint16_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        int16_t real;
        uint16_t base;
      } u_v_x;
      u_v_x.real = this->v_x;
      *(outbuffer + offset + 0) = (u_v_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_x.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->v_x);
      union {
        int16_t real;
        uint16_t base;
      } u_v_y;
      u_v_y.real = this->v_y;
      *(outbuffer + offset + 0) = (u_v_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_y.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->v_y);
      union {
        int16_t real;
        uint16_t base;
      } u_v_theta;
      u_v_theta.real = this->v_theta;
      *(outbuffer + offset + 0) = (u_v_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_theta.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->v_theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        int16_t real;
        uint16_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        int16_t real;
        uint16_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        int16_t real;
        uint16_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        int16_t real;
        uint16_t base;
      } u_v_x;
      u_v_x.base = 0;
      u_v_x.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_x.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->v_x = u_v_x.real;
      offset += sizeof(this->v_x);
      union {
        int16_t real;
        uint16_t base;
      } u_v_y;
      u_v_y.base = 0;
      u_v_y.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_y.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->v_y = u_v_y.real;
      offset += sizeof(this->v_y);
      union {
        int16_t real;
        uint16_t base;
      } u_v_theta;
      u_v_theta.base = 0;
      u_v_theta.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_theta.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->v_theta = u_v_theta.real;
      offset += sizeof(this->v_theta);
     return offset;
    }

    virtual const char * getType() override { return "tsuten_msgs/Odometry"; };
    virtual const char * getMD5() override { return "33960ec3c88f6fb2cf1713ab25eed8a0"; };

  };

}
#endif
