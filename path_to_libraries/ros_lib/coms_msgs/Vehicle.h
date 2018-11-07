#ifndef _ROS_coms_msgs_Vehicle_h
#define _ROS_coms_msgs_Vehicle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/time.h"

namespace coms_msgs
{

  class Vehicle : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef ros::Time _headerstamp_type;
      _headerstamp_type headerstamp;
      typedef const char* _model_type;
      _model_type model;
      typedef const char* _charger_type_type;
      _charger_type_type charger_type;
      typedef float _battery_charge_type;
      _battery_charge_type battery_charge;
      typedef int32_t _charge_level_type;
      _charge_level_type charge_level;

    Vehicle():
      header(),
      headerstamp(),
      model(""),
      charger_type(""),
      battery_charge(0),
      charge_level(0)
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
      uint32_t length_model = strlen(this->model);
      varToArr(outbuffer + offset, length_model);
      offset += 4;
      memcpy(outbuffer + offset, this->model, length_model);
      offset += length_model;
      uint32_t length_charger_type = strlen(this->charger_type);
      varToArr(outbuffer + offset, length_charger_type);
      offset += 4;
      memcpy(outbuffer + offset, this->charger_type, length_charger_type);
      offset += length_charger_type;
      union {
        float real;
        uint32_t base;
      } u_battery_charge;
      u_battery_charge.real = this->battery_charge;
      *(outbuffer + offset + 0) = (u_battery_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery_charge);
      union {
        int32_t real;
        uint32_t base;
      } u_charge_level;
      u_charge_level.real = this->charge_level;
      *(outbuffer + offset + 0) = (u_charge_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charge_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charge_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->charge_level);
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
      uint32_t length_model;
      arrToVar(length_model, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model-1]=0;
      this->model = (char *)(inbuffer + offset-1);
      offset += length_model;
      uint32_t length_charger_type;
      arrToVar(length_charger_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_charger_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_charger_type-1]=0;
      this->charger_type = (char *)(inbuffer + offset-1);
      offset += length_charger_type;
      union {
        float real;
        uint32_t base;
      } u_battery_charge;
      u_battery_charge.base = 0;
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery_charge = u_battery_charge.real;
      offset += sizeof(this->battery_charge);
      union {
        int32_t real;
        uint32_t base;
      } u_charge_level;
      u_charge_level.base = 0;
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charge_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->charge_level = u_charge_level.real;
      offset += sizeof(this->charge_level);
     return offset;
    }

    const char * getType(){ return "coms_msgs/Vehicle"; };
    const char * getMD5(){ return "749a40e9e950fe32af39599fa5902eb8"; };

  };

}
#endif
