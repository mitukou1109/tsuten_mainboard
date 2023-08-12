#ifndef _ROS_SERVICE_CorrectRobotPose_h
#define _ROS_SERVICE_CorrectRobotPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace tsuten_msgs
{

static const char CORRECTROBOTPOSE[] = "tsuten_msgs/CorrectRobotPose";

  class CorrectRobotPoseRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _robot_pose_type;
      _robot_pose_type robot_pose;

    CorrectRobotPoseRequest():
      robot_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->robot_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->robot_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return CORRECTROBOTPOSE; };
    virtual const char * getMD5() override { return "bb46cd500e029a262f0c2284fecd8ed7"; };

  };

  class CorrectRobotPoseResponse : public ros::Msg
  {
    public:

    CorrectRobotPoseResponse()
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

    virtual const char * getType() override { return CORRECTROBOTPOSE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class CorrectRobotPose {
    public:
    typedef CorrectRobotPoseRequest Request;
    typedef CorrectRobotPoseResponse Response;
  };

}
#endif
