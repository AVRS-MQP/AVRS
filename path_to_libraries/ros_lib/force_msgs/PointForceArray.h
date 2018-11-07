#ifndef _ROS_force_msgs_PointForceArray_h
#define _ROS_force_msgs_PointForceArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "force_msgs/PointForce.h"

namespace force_msgs
{

  class PointForceArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t PointForces_length;
      typedef force_msgs::PointForce _PointForces_type;
      _PointForces_type st_PointForces;
      _PointForces_type * PointForces;

    PointForceArray():
      header(),
      PointForces_length(0), PointForces(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->PointForces_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->PointForces_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->PointForces_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->PointForces_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->PointForces_length);
      for( uint32_t i = 0; i < PointForces_length; i++){
      offset += this->PointForces[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t PointForces_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      PointForces_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      PointForces_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      PointForces_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->PointForces_length);
      if(PointForces_lengthT > PointForces_length)
        this->PointForces = (force_msgs::PointForce*)realloc(this->PointForces, PointForces_lengthT * sizeof(force_msgs::PointForce));
      PointForces_length = PointForces_lengthT;
      for( uint32_t i = 0; i < PointForces_length; i++){
      offset += this->st_PointForces.deserialize(inbuffer + offset);
        memcpy( &(this->PointForces[i]), &(this->st_PointForces), sizeof(force_msgs::PointForce));
      }
     return offset;
    }

    const char * getType(){ return "force_msgs/PointForceArray"; };
    const char * getMD5(){ return "478e357e8e0b95825693d0091cff5ff9"; };

  };

}
#endif
