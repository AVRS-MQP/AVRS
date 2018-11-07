#ifndef _ROS_ur_msgs_RobotStateRTMsg_h
#define _ROS_ur_msgs_RobotStateRTMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

  class RobotStateRTMsg : public ros::Msg
  {
    public:
      typedef double _time_type;
      _time_type time;
      uint32_t q_target_length;
      typedef double _q_target_type;
      _q_target_type st_q_target;
      _q_target_type * q_target;
      uint32_t qd_target_length;
      typedef double _qd_target_type;
      _qd_target_type st_qd_target;
      _qd_target_type * qd_target;
      uint32_t qdd_target_length;
      typedef double _qdd_target_type;
      _qdd_target_type st_qdd_target;
      _qdd_target_type * qdd_target;
      uint32_t i_target_length;
      typedef double _i_target_type;
      _i_target_type st_i_target;
      _i_target_type * i_target;
      uint32_t m_target_length;
      typedef double _m_target_type;
      _m_target_type st_m_target;
      _m_target_type * m_target;
      uint32_t q_actual_length;
      typedef double _q_actual_type;
      _q_actual_type st_q_actual;
      _q_actual_type * q_actual;
      uint32_t qd_actual_length;
      typedef double _qd_actual_type;
      _qd_actual_type st_qd_actual;
      _qd_actual_type * qd_actual;
      uint32_t i_actual_length;
      typedef double _i_actual_type;
      _i_actual_type st_i_actual;
      _i_actual_type * i_actual;
      uint32_t tool_acc_values_length;
      typedef double _tool_acc_values_type;
      _tool_acc_values_type st_tool_acc_values;
      _tool_acc_values_type * tool_acc_values;
      uint32_t tcp_force_length;
      typedef double _tcp_force_type;
      _tcp_force_type st_tcp_force;
      _tcp_force_type * tcp_force;
      uint32_t tool_vector_length;
      typedef double _tool_vector_type;
      _tool_vector_type st_tool_vector;
      _tool_vector_type * tool_vector;
      uint32_t tcp_speed_length;
      typedef double _tcp_speed_type;
      _tcp_speed_type st_tcp_speed;
      _tcp_speed_type * tcp_speed;
      typedef double _digital_input_bits_type;
      _digital_input_bits_type digital_input_bits;
      uint32_t motor_temperatures_length;
      typedef double _motor_temperatures_type;
      _motor_temperatures_type st_motor_temperatures;
      _motor_temperatures_type * motor_temperatures;
      typedef double _controller_timer_type;
      _controller_timer_type controller_timer;
      typedef double _test_value_type;
      _test_value_type test_value;
      typedef double _robot_mode_type;
      _robot_mode_type robot_mode;
      uint32_t joint_modes_length;
      typedef double _joint_modes_type;
      _joint_modes_type st_joint_modes;
      _joint_modes_type * joint_modes;

    RobotStateRTMsg():
      time(0),
      q_target_length(0), q_target(NULL),
      qd_target_length(0), qd_target(NULL),
      qdd_target_length(0), qdd_target(NULL),
      i_target_length(0), i_target(NULL),
      m_target_length(0), m_target(NULL),
      q_actual_length(0), q_actual(NULL),
      qd_actual_length(0), qd_actual(NULL),
      i_actual_length(0), i_actual(NULL),
      tool_acc_values_length(0), tool_acc_values(NULL),
      tcp_force_length(0), tcp_force(NULL),
      tool_vector_length(0), tool_vector(NULL),
      tcp_speed_length(0), tcp_speed(NULL),
      digital_input_bits(0),
      motor_temperatures_length(0), motor_temperatures(NULL),
      controller_timer(0),
      test_value(0),
      robot_mode(0),
      joint_modes_length(0), joint_modes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      *(outbuffer + offset + 0) = (this->q_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->q_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->q_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->q_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q_target_length);
      for( uint32_t i = 0; i < q_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_q_targeti;
      u_q_targeti.real = this->q_target[i];
      *(outbuffer + offset + 0) = (u_q_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_q_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_q_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_q_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_q_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->q_target[i]);
      }
      *(outbuffer + offset + 0) = (this->qd_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qd_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qd_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qd_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qd_target_length);
      for( uint32_t i = 0; i < qd_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_qd_targeti;
      u_qd_targeti.real = this->qd_target[i];
      *(outbuffer + offset + 0) = (u_qd_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qd_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qd_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qd_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qd_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qd_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qd_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qd_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qd_target[i]);
      }
      *(outbuffer + offset + 0) = (this->qdd_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qdd_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qdd_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qdd_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qdd_target_length);
      for( uint32_t i = 0; i < qdd_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_qdd_targeti;
      u_qdd_targeti.real = this->qdd_target[i];
      *(outbuffer + offset + 0) = (u_qdd_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qdd_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qdd_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qdd_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qdd_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qdd_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qdd_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qdd_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qdd_target[i]);
      }
      *(outbuffer + offset + 0) = (this->i_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_target_length);
      for( uint32_t i = 0; i < i_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_i_targeti;
      u_i_targeti.real = this->i_target[i];
      *(outbuffer + offset + 0) = (u_i_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_target[i]);
      }
      *(outbuffer + offset + 0) = (this->m_target_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->m_target_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->m_target_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->m_target_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m_target_length);
      for( uint32_t i = 0; i < m_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_m_targeti;
      u_m_targeti.real = this->m_target[i];
      *(outbuffer + offset + 0) = (u_m_targeti.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m_targeti.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m_targeti.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m_targeti.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m_targeti.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m_targeti.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m_targeti.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m_targeti.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m_target[i]);
      }
      *(outbuffer + offset + 0) = (this->q_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->q_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->q_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->q_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->q_actual_length);
      for( uint32_t i = 0; i < q_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_q_actuali;
      u_q_actuali.real = this->q_actual[i];
      *(outbuffer + offset + 0) = (u_q_actuali.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_q_actuali.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_q_actuali.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_q_actuali.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_q_actuali.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_q_actuali.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_q_actuali.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_q_actuali.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->q_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->qd_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->qd_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->qd_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->qd_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->qd_actual_length);
      for( uint32_t i = 0; i < qd_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_qd_actuali;
      u_qd_actuali.real = this->qd_actual[i];
      *(outbuffer + offset + 0) = (u_qd_actuali.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_qd_actuali.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_qd_actuali.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_qd_actuali.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_qd_actuali.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_qd_actuali.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_qd_actuali.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_qd_actuali.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->qd_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->i_actual_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->i_actual_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->i_actual_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->i_actual_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_actual_length);
      for( uint32_t i = 0; i < i_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_i_actuali;
      u_i_actuali.real = this->i_actual[i];
      *(outbuffer + offset + 0) = (u_i_actuali.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_actuali.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_actuali.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_actuali.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_actuali.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_actuali.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_actuali.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_actuali.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_actual[i]);
      }
      *(outbuffer + offset + 0) = (this->tool_acc_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tool_acc_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tool_acc_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tool_acc_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_acc_values_length);
      for( uint32_t i = 0; i < tool_acc_values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tool_acc_valuesi;
      u_tool_acc_valuesi.real = this->tool_acc_values[i];
      *(outbuffer + offset + 0) = (u_tool_acc_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tool_acc_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tool_acc_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tool_acc_valuesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tool_acc_valuesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tool_acc_valuesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tool_acc_valuesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tool_acc_valuesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tool_acc_values[i]);
      }
      *(outbuffer + offset + 0) = (this->tcp_force_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tcp_force_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tcp_force_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tcp_force_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tcp_force_length);
      for( uint32_t i = 0; i < tcp_force_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tcp_forcei;
      u_tcp_forcei.real = this->tcp_force[i];
      *(outbuffer + offset + 0) = (u_tcp_forcei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tcp_forcei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tcp_forcei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tcp_forcei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tcp_forcei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tcp_forcei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tcp_forcei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tcp_forcei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tcp_force[i]);
      }
      *(outbuffer + offset + 0) = (this->tool_vector_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tool_vector_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tool_vector_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tool_vector_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_vector_length);
      for( uint32_t i = 0; i < tool_vector_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tool_vectori;
      u_tool_vectori.real = this->tool_vector[i];
      *(outbuffer + offset + 0) = (u_tool_vectori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tool_vectori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tool_vectori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tool_vectori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tool_vectori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tool_vectori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tool_vectori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tool_vectori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tool_vector[i]);
      }
      *(outbuffer + offset + 0) = (this->tcp_speed_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tcp_speed_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tcp_speed_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tcp_speed_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tcp_speed_length);
      for( uint32_t i = 0; i < tcp_speed_length; i++){
      union {
        double real;
        uint64_t base;
      } u_tcp_speedi;
      u_tcp_speedi.real = this->tcp_speed[i];
      *(outbuffer + offset + 0) = (u_tcp_speedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tcp_speedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tcp_speedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tcp_speedi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tcp_speedi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tcp_speedi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tcp_speedi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tcp_speedi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tcp_speed[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_digital_input_bits;
      u_digital_input_bits.real = this->digital_input_bits;
      *(outbuffer + offset + 0) = (u_digital_input_bits.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_digital_input_bits.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_digital_input_bits.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_digital_input_bits.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_digital_input_bits.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_digital_input_bits.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_digital_input_bits.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_digital_input_bits.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->digital_input_bits);
      *(outbuffer + offset + 0) = (this->motor_temperatures_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_temperatures_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_temperatures_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_temperatures_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_temperatures_length);
      for( uint32_t i = 0; i < motor_temperatures_length; i++){
      union {
        double real;
        uint64_t base;
      } u_motor_temperaturesi;
      u_motor_temperaturesi.real = this->motor_temperatures[i];
      *(outbuffer + offset + 0) = (u_motor_temperaturesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_temperaturesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_temperaturesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_temperaturesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_motor_temperaturesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_motor_temperaturesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_motor_temperaturesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_motor_temperaturesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->motor_temperatures[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_controller_timer;
      u_controller_timer.real = this->controller_timer;
      *(outbuffer + offset + 0) = (u_controller_timer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_controller_timer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_controller_timer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_controller_timer.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_controller_timer.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_controller_timer.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_controller_timer.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_controller_timer.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->controller_timer);
      union {
        double real;
        uint64_t base;
      } u_test_value;
      u_test_value.real = this->test_value;
      *(outbuffer + offset + 0) = (u_test_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_test_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_test_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_test_value.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_test_value.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_test_value.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_test_value.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_test_value.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->test_value);
      union {
        double real;
        uint64_t base;
      } u_robot_mode;
      u_robot_mode.real = this->robot_mode;
      *(outbuffer + offset + 0) = (u_robot_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_mode.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_robot_mode.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_robot_mode.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_robot_mode.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_robot_mode.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->robot_mode);
      *(outbuffer + offset + 0) = (this->joint_modes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_modes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_modes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_modes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_modes_length);
      for( uint32_t i = 0; i < joint_modes_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_modesi;
      u_joint_modesi.real = this->joint_modes[i];
      *(outbuffer + offset + 0) = (u_joint_modesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_modesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_modesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_modesi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_modesi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_modesi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_modesi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_modesi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_modes[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
      uint32_t q_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      q_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->q_target_length);
      if(q_target_lengthT > q_target_length)
        this->q_target = (double*)realloc(this->q_target, q_target_lengthT * sizeof(double));
      q_target_length = q_target_lengthT;
      for( uint32_t i = 0; i < q_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_q_target;
      u_st_q_target.base = 0;
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_q_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_q_target = u_st_q_target.real;
      offset += sizeof(this->st_q_target);
        memcpy( &(this->q_target[i]), &(this->st_q_target), sizeof(double));
      }
      uint32_t qd_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qd_target_length);
      if(qd_target_lengthT > qd_target_length)
        this->qd_target = (double*)realloc(this->qd_target, qd_target_lengthT * sizeof(double));
      qd_target_length = qd_target_lengthT;
      for( uint32_t i = 0; i < qd_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_qd_target;
      u_st_qd_target.base = 0;
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_qd_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_qd_target = u_st_qd_target.real;
      offset += sizeof(this->st_qd_target);
        memcpy( &(this->qd_target[i]), &(this->st_qd_target), sizeof(double));
      }
      uint32_t qdd_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qdd_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qdd_target_length);
      if(qdd_target_lengthT > qdd_target_length)
        this->qdd_target = (double*)realloc(this->qdd_target, qdd_target_lengthT * sizeof(double));
      qdd_target_length = qdd_target_lengthT;
      for( uint32_t i = 0; i < qdd_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_qdd_target;
      u_st_qdd_target.base = 0;
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_qdd_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_qdd_target = u_st_qdd_target.real;
      offset += sizeof(this->st_qdd_target);
        memcpy( &(this->qdd_target[i]), &(this->st_qdd_target), sizeof(double));
      }
      uint32_t i_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_target_length);
      if(i_target_lengthT > i_target_length)
        this->i_target = (double*)realloc(this->i_target, i_target_lengthT * sizeof(double));
      i_target_length = i_target_lengthT;
      for( uint32_t i = 0; i < i_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_i_target;
      u_st_i_target.base = 0;
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_i_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_i_target = u_st_i_target.real;
      offset += sizeof(this->st_i_target);
        memcpy( &(this->i_target[i]), &(this->st_i_target), sizeof(double));
      }
      uint32_t m_target_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      m_target_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->m_target_length);
      if(m_target_lengthT > m_target_length)
        this->m_target = (double*)realloc(this->m_target, m_target_lengthT * sizeof(double));
      m_target_length = m_target_lengthT;
      for( uint32_t i = 0; i < m_target_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_m_target;
      u_st_m_target.base = 0;
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_m_target.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_m_target = u_st_m_target.real;
      offset += sizeof(this->st_m_target);
        memcpy( &(this->m_target[i]), &(this->st_m_target), sizeof(double));
      }
      uint32_t q_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      q_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->q_actual_length);
      if(q_actual_lengthT > q_actual_length)
        this->q_actual = (double*)realloc(this->q_actual, q_actual_lengthT * sizeof(double));
      q_actual_length = q_actual_lengthT;
      for( uint32_t i = 0; i < q_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_q_actual;
      u_st_q_actual.base = 0;
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_q_actual.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_q_actual = u_st_q_actual.real;
      offset += sizeof(this->st_q_actual);
        memcpy( &(this->q_actual[i]), &(this->st_q_actual), sizeof(double));
      }
      uint32_t qd_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      qd_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->qd_actual_length);
      if(qd_actual_lengthT > qd_actual_length)
        this->qd_actual = (double*)realloc(this->qd_actual, qd_actual_lengthT * sizeof(double));
      qd_actual_length = qd_actual_lengthT;
      for( uint32_t i = 0; i < qd_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_qd_actual;
      u_st_qd_actual.base = 0;
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_qd_actual.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_qd_actual = u_st_qd_actual.real;
      offset += sizeof(this->st_qd_actual);
        memcpy( &(this->qd_actual[i]), &(this->st_qd_actual), sizeof(double));
      }
      uint32_t i_actual_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      i_actual_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->i_actual_length);
      if(i_actual_lengthT > i_actual_length)
        this->i_actual = (double*)realloc(this->i_actual, i_actual_lengthT * sizeof(double));
      i_actual_length = i_actual_lengthT;
      for( uint32_t i = 0; i < i_actual_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_i_actual;
      u_st_i_actual.base = 0;
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_i_actual.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_i_actual = u_st_i_actual.real;
      offset += sizeof(this->st_i_actual);
        memcpy( &(this->i_actual[i]), &(this->st_i_actual), sizeof(double));
      }
      uint32_t tool_acc_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tool_acc_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tool_acc_values_length);
      if(tool_acc_values_lengthT > tool_acc_values_length)
        this->tool_acc_values = (double*)realloc(this->tool_acc_values, tool_acc_values_lengthT * sizeof(double));
      tool_acc_values_length = tool_acc_values_lengthT;
      for( uint32_t i = 0; i < tool_acc_values_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tool_acc_values;
      u_st_tool_acc_values.base = 0;
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tool_acc_values.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tool_acc_values = u_st_tool_acc_values.real;
      offset += sizeof(this->st_tool_acc_values);
        memcpy( &(this->tool_acc_values[i]), &(this->st_tool_acc_values), sizeof(double));
      }
      uint32_t tcp_force_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tcp_force_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tcp_force_length);
      if(tcp_force_lengthT > tcp_force_length)
        this->tcp_force = (double*)realloc(this->tcp_force, tcp_force_lengthT * sizeof(double));
      tcp_force_length = tcp_force_lengthT;
      for( uint32_t i = 0; i < tcp_force_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tcp_force;
      u_st_tcp_force.base = 0;
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tcp_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tcp_force = u_st_tcp_force.real;
      offset += sizeof(this->st_tcp_force);
        memcpy( &(this->tcp_force[i]), &(this->st_tcp_force), sizeof(double));
      }
      uint32_t tool_vector_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tool_vector_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tool_vector_length);
      if(tool_vector_lengthT > tool_vector_length)
        this->tool_vector = (double*)realloc(this->tool_vector, tool_vector_lengthT * sizeof(double));
      tool_vector_length = tool_vector_lengthT;
      for( uint32_t i = 0; i < tool_vector_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tool_vector;
      u_st_tool_vector.base = 0;
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tool_vector.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tool_vector = u_st_tool_vector.real;
      offset += sizeof(this->st_tool_vector);
        memcpy( &(this->tool_vector[i]), &(this->st_tool_vector), sizeof(double));
      }
      uint32_t tcp_speed_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tcp_speed_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tcp_speed_length);
      if(tcp_speed_lengthT > tcp_speed_length)
        this->tcp_speed = (double*)realloc(this->tcp_speed, tcp_speed_lengthT * sizeof(double));
      tcp_speed_length = tcp_speed_lengthT;
      for( uint32_t i = 0; i < tcp_speed_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_tcp_speed;
      u_st_tcp_speed.base = 0;
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_tcp_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_tcp_speed = u_st_tcp_speed.real;
      offset += sizeof(this->st_tcp_speed);
        memcpy( &(this->tcp_speed[i]), &(this->st_tcp_speed), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_digital_input_bits;
      u_digital_input_bits.base = 0;
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_digital_input_bits.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->digital_input_bits = u_digital_input_bits.real;
      offset += sizeof(this->digital_input_bits);
      uint32_t motor_temperatures_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_temperatures_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_temperatures_length);
      if(motor_temperatures_lengthT > motor_temperatures_length)
        this->motor_temperatures = (double*)realloc(this->motor_temperatures, motor_temperatures_lengthT * sizeof(double));
      motor_temperatures_length = motor_temperatures_lengthT;
      for( uint32_t i = 0; i < motor_temperatures_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_motor_temperatures;
      u_st_motor_temperatures.base = 0;
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_motor_temperatures.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_motor_temperatures = u_st_motor_temperatures.real;
      offset += sizeof(this->st_motor_temperatures);
        memcpy( &(this->motor_temperatures[i]), &(this->st_motor_temperatures), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_controller_timer;
      u_controller_timer.base = 0;
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_controller_timer.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->controller_timer = u_controller_timer.real;
      offset += sizeof(this->controller_timer);
      union {
        double real;
        uint64_t base;
      } u_test_value;
      u_test_value.base = 0;
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_test_value.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->test_value = u_test_value.real;
      offset += sizeof(this->test_value);
      union {
        double real;
        uint64_t base;
      } u_robot_mode;
      u_robot_mode.base = 0;
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_robot_mode.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->robot_mode = u_robot_mode.real;
      offset += sizeof(this->robot_mode);
      uint32_t joint_modes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_modes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_modes_length);
      if(joint_modes_lengthT > joint_modes_length)
        this->joint_modes = (double*)realloc(this->joint_modes, joint_modes_lengthT * sizeof(double));
      joint_modes_length = joint_modes_lengthT;
      for( uint32_t i = 0; i < joint_modes_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_modes;
      u_st_joint_modes.base = 0;
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_modes.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_modes = u_st_joint_modes.real;
      offset += sizeof(this->st_joint_modes);
        memcpy( &(this->joint_modes[i]), &(this->st_joint_modes), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "ur_msgs/RobotStateRTMsg"; };
    const char * getMD5(){ return "ce6feddd3ccb4ca7dbcd0ff105b603c7"; };

  };

}
#endif
