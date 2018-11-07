#ifndef _ROS_coms_msgs_Station_h
#define _ROS_coms_msgs_Station_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace coms_msgs
{

  class Station : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ros::Time _headerstamp_type;
      _headerstamp_type headerstamp;
      typedef float _xMax_type;
      _xMax_type xMax;
      typedef float _xMin_type;
      _xMin_type xMin;
      typedef float _yMax_type;
      _yMax_type yMax;
      typedef float _yMin_type;
      _yMin_type yMin;
      typedef float _zMax_type;
      _zMax_type zMax;
      typedef float _zMin_type;
      _zMin_type zMin;

    Station():
      header(),
      headerstamp(),
      xMax(0),
      xMin(0),
      yMax(0),
      yMin(0),
      zMax(0),
      zMin(0)
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
        float real;
        uint32_t base;
      } u_xMax;
      u_xMax.real = this->xMax;
      *(outbuffer + offset + 0) = (u_xMax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xMax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xMax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xMax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xMax);
      union {
        float real;
        uint32_t base;
      } u_xMin;
      u_xMin.real = this->xMin;
      *(outbuffer + offset + 0) = (u_xMin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xMin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xMin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xMin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xMin);
      union {
        float real;
        uint32_t base;
      } u_yMax;
      u_yMax.real = this->yMax;
      *(outbuffer + offset + 0) = (u_yMax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yMax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yMax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yMax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yMax);
      union {
        float real;
        uint32_t base;
      } u_yMin;
      u_yMin.real = this->yMin;
      *(outbuffer + offset + 0) = (u_yMin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yMin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yMin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yMin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yMin);
      union {
        float real;
        uint32_t base;
      } u_zMax;
      u_zMax.real = this->zMax;
      *(outbuffer + offset + 0) = (u_zMax.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zMax.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zMax.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zMax.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zMax);
      union {
        float real;
        uint32_t base;
      } u_zMin;
      u_zMin.real = this->zMin;
      *(outbuffer + offset + 0) = (u_zMin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zMin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zMin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zMin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zMin);
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
        float real;
        uint32_t base;
      } u_xMax;
      u_xMax.base = 0;
      u_xMax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xMax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xMax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xMax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xMax = u_xMax.real;
      offset += sizeof(this->xMax);
      union {
        float real;
        uint32_t base;
      } u_xMin;
      u_xMin.base = 0;
      u_xMin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xMin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xMin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xMin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xMin = u_xMin.real;
      offset += sizeof(this->xMin);
      union {
        float real;
        uint32_t base;
      } u_yMax;
      u_yMax.base = 0;
      u_yMax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yMax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yMax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yMax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yMax = u_yMax.real;
      offset += sizeof(this->yMax);
      union {
        float real;
        uint32_t base;
      } u_yMin;
      u_yMin.base = 0;
      u_yMin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yMin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yMin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yMin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yMin = u_yMin.real;
      offset += sizeof(this->yMin);
      union {
        float real;
        uint32_t base;
      } u_zMax;
      u_zMax.base = 0;
      u_zMax.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zMax.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zMax.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zMax.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zMax = u_zMax.real;
      offset += sizeof(this->zMax);
      union {
        float real;
        uint32_t base;
      } u_zMin;
      u_zMin.base = 0;
      u_zMin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zMin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zMin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zMin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zMin = u_zMin.real;
      offset += sizeof(this->zMin);
     return offset;
    }

    const char * getType(){ return "coms_msgs/Station"; };
    const char * getMD5(){ return "147bfffe45a561e191aec9dbdf4be67e"; };

  };

}
#endif
