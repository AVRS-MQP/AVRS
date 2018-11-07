#ifndef _ROS_force_msgs_PointForce_h
#define _ROS_force_msgs_PointForce_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace force_msgs
{

  class PointForce : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ros::Time _headerstamp_type;
      _headerstamp_type headerstamp;
      typedef double _xLoc_type;
      _xLoc_type xLoc;
      typedef double _yLoc_type;
      _yLoc_type yLoc;
      typedef double _zLoc_type;
      _zLoc_type zLoc;
      typedef double _xForce_type;
      _xForce_type xForce;
      typedef double _yForce_type;
      _yForce_type yForce;
      typedef double _zForce_type;
      _zForce_type zForce;

    PointForce():
      header(),
      headerstamp(),
      xLoc(0),
      yLoc(0),
      zLoc(0),
      xForce(0),
      yForce(0),
      zForce(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->headerstamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->headerstamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->headerstamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->headerstamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headerstamp.sec);
      *(outbuffer + offset + 0) = (this->headerstamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->headerstamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->headerstamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->headerstamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headerstamp.nsec);
      union {
        double real;
        uint64_t base;
      } u_xLoc;
      u_xLoc.real = this->xLoc;
      *(outbuffer + offset + 0) = (u_xLoc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xLoc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xLoc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xLoc.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xLoc.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xLoc.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xLoc.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xLoc.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->xLoc);
      union {
        double real;
        uint64_t base;
      } u_yLoc;
      u_yLoc.real = this->yLoc;
      *(outbuffer + offset + 0) = (u_yLoc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yLoc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yLoc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yLoc.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yLoc.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yLoc.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yLoc.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yLoc.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yLoc);
      union {
        double real;
        uint64_t base;
      } u_zLoc;
      u_zLoc.real = this->zLoc;
      *(outbuffer + offset + 0) = (u_zLoc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zLoc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zLoc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zLoc.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_zLoc.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_zLoc.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_zLoc.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_zLoc.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->zLoc);
      union {
        double real;
        uint64_t base;
      } u_xForce;
      u_xForce.real = this->xForce;
      *(outbuffer + offset + 0) = (u_xForce.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xForce.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xForce.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xForce.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xForce.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xForce.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xForce.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xForce.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->xForce);
      union {
        double real;
        uint64_t base;
      } u_yForce;
      u_yForce.real = this->yForce;
      *(outbuffer + offset + 0) = (u_yForce.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yForce.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yForce.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yForce.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yForce.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yForce.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yForce.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yForce.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yForce);
      union {
        double real;
        uint64_t base;
      } u_zForce;
      u_zForce.real = this->zForce;
      *(outbuffer + offset + 0) = (u_zForce.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zForce.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zForce.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zForce.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_zForce.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_zForce.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_zForce.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_zForce.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->zForce);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->headerstamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->headerstamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->headerstamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->headerstamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->headerstamp.sec);
      this->headerstamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->headerstamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->headerstamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->headerstamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->headerstamp.nsec);
      union {
        double real;
        uint64_t base;
      } u_xLoc;
      u_xLoc.base = 0;
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_xLoc.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->xLoc = u_xLoc.real;
      offset += sizeof(this->xLoc);
      union {
        double real;
        uint64_t base;
      } u_yLoc;
      u_yLoc.base = 0;
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yLoc.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yLoc = u_yLoc.real;
      offset += sizeof(this->yLoc);
      union {
        double real;
        uint64_t base;
      } u_zLoc;
      u_zLoc.base = 0;
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_zLoc.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->zLoc = u_zLoc.real;
      offset += sizeof(this->zLoc);
      union {
        double real;
        uint64_t base;
      } u_xForce;
      u_xForce.base = 0;
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_xForce.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->xForce = u_xForce.real;
      offset += sizeof(this->xForce);
      union {
        double real;
        uint64_t base;
      } u_yForce;
      u_yForce.base = 0;
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yForce.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yForce = u_yForce.real;
      offset += sizeof(this->yForce);
      union {
        double real;
        uint64_t base;
      } u_zForce;
      u_zForce.base = 0;
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_zForce.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->zForce = u_zForce.real;
      offset += sizeof(this->zForce);
     return offset;
    }

    const char * getType(){ return "force_msgs/PointForce"; };
    const char * getMD5(){ return "574692728549b4b4cb899072da514e35"; };

  };

}
#endif
