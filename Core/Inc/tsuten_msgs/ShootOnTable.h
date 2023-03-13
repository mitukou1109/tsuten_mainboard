#ifndef _ROS_SERVICE_ShootOnTable_h
#define _ROS_SERVICE_ShootOnTable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tsuten_msgs
{

static const char SHOOTONTABLE[] = "tsuten_msgs/ShootOnTable";

  class ShootOnTableRequest : public ros::Msg
  {
    public:
      typedef uint8_t _table_type;
      _table_type table;
      enum { DUAL_TABLE_UPPER =  0 };
      enum { DUAL_TABLE_LOWER =  1 };
      enum { MOVABLE_TABLE_1200 =  2 };
      enum { MOVABLE_TABLE_1500 =  3 };
      enum { MOVABLE_TABLE_1800 =  4 };

    ShootOnTableRequest():
      table(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->table >> (8 * 0)) & 0xFF;
      offset += sizeof(this->table);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->table =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->table);
     return offset;
    }

    virtual const char * getType() override { return SHOOTONTABLE; };
    virtual const char * getMD5() override { return "e1b7b87bce265f65c513c23b471cd72d"; };

  };

  class ShootOnTableResponse : public ros::Msg
  {
    public:

    ShootOnTableResponse()
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

    virtual const char * getType() override { return SHOOTONTABLE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ShootOnTable {
    public:
    typedef ShootOnTableRequest Request;
    typedef ShootOnTableResponse Response;
  };

}
#endif
