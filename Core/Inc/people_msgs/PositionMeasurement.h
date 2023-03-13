#ifndef _ROS_people_msgs_PositionMeasurement_h
#define _ROS_people_msgs_PositionMeasurement_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace people_msgs
{

  class PositionMeasurement : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _object_id_type;
      _object_id_type object_id;
      typedef geometry_msgs::Point _pos_type;
      _pos_type pos;
      typedef double _reliability_type;
      _reliability_type reliability;
      double covariance[9];
      typedef int8_t _initialization_type;
      _initialization_type initialization;

    PositionMeasurement():
      header(),
      name(""),
      object_id(""),
      pos(),
      reliability(0),
      covariance(),
      initialization(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_object_id = strlen(this->object_id);
      varToArr(outbuffer + offset, length_object_id);
      offset += 4;
      memcpy(outbuffer + offset, this->object_id, length_object_id);
      offset += length_object_id;
      offset += this->pos.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_reliability;
      u_reliability.real = this->reliability;
      *(outbuffer + offset + 0) = (u_reliability.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_reliability.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_reliability.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_reliability.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_reliability.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_reliability.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_reliability.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_reliability.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->reliability);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_covariancei;
      u_covariancei.real = this->covariance[i];
      *(outbuffer + offset + 0) = (u_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_covariancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_covariancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_covariancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_covariancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_covariancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->covariance[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_initialization;
      u_initialization.real = this->initialization;
      *(outbuffer + offset + 0) = (u_initialization.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->initialization);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_object_id;
      arrToVar(length_object_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_id-1]=0;
      this->object_id = (char *)(inbuffer + offset-1);
      offset += length_object_id;
      offset += this->pos.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_reliability;
      u_reliability.base = 0;
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_reliability.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->reliability = u_reliability.real;
      offset += sizeof(this->reliability);
      for( uint32_t i = 0; i < 9; i++){
      union {
        double real;
        uint64_t base;
      } u_covariancei;
      u_covariancei.base = 0;
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_covariancei.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->covariance[i] = u_covariancei.real;
      offset += sizeof(this->covariance[i]);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_initialization;
      u_initialization.base = 0;
      u_initialization.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->initialization = u_initialization.real;
      offset += sizeof(this->initialization);
     return offset;
    }

    virtual const char * getType() override { return "people_msgs/PositionMeasurement"; };
    virtual const char * getMD5() override { return "54fa938b4ec28728e01575b79eb0ec7c"; };

  };

}
#endif
