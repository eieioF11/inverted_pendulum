#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/dynamixel_utils.hpp"
// control
#include "control/pid_control.hpp"
// filter
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"

#define DEBUG_SERIAL Serial
HardwareSerial &DXL_SERIAL = Serial1;

// M5Stack Core2 PORT A
const uint8_t RX_SERVO = 9; // 32
const uint8_t TX_SERVO = 8; // 33

// 右 id
const uint8_t DXL_ID_LW = 1;
const uint8_t DXL_ID_LF = 3;
const uint8_t DXL_ID_LR = 4;
// 左 id
const uint8_t DXL_ID_RW = 2;
const uint8_t DXL_ID_RF = 5;
const uint8_t DXL_ID_RR = 6;

const float DXL_PROTOCOL_VERSION = 2.0;

// const float MAX_RPM = 101.0;// M288
const float MAX_RPM = 370.0; // M077
const float MIN_ANGLE_LIMIT = -10.0 * DEG_TO_RAD;
const float MAX_ANGLE_LIMIT = 90.0 * DEG_TO_RAD;

const float LPF_ALPHA = 0.88;
const float COMP_ALPHA = 0.88;

// モーターの設定
// mode
//  OP_CURRENT
//  OP_VELOCITY
//  OP_POSITION
//  OP_EXTENDED_POSITION
//  OP_CURRENT_BASED_POSITION
//  OP_PWM
DXLMotor m_lw(DXL_ID_LW, OP_VELOCITY);
DXLMotor m_lf(DXL_ID_LF, OP_CURRENT_BASED_POSITION);
DXLMotor m_lr(DXL_ID_LR, OP_CURRENT_BASED_POSITION);
DXLMotor m_rw(DXL_ID_RW, OP_VELOCITY);
DXLMotor m_rf(DXL_ID_RF, OP_CURRENT_BASED_POSITION);
DXLMotor m_rr(DXL_ID_RR, OP_CURRENT_BASED_POSITION);

// set dxl init pos
void set_dxl_init_pos()
{
  m_lf.move(0.0);
  m_rf.move(0.0);
  m_lr.move(0.0);
  m_rr.move(0.0);
}

// filter
common_lib::LowpassFilterf lpf_acc_x(LPF_ALPHA);
common_lib::LowpassFilterf lpf_acc_y(LPF_ALPHA);
common_lib::ComplementaryFilterf comp_filter_x(COMP_ALPHA);
common_lib::ComplementaryFilterf comp_filter_y(COMP_ALPHA);

// pid parameter setting
common_lib::PidControl pid;
common_lib::pid_parameter_t pid_param;
enum class sel_param_t
{
  KP,
  KI,
  KD
};
sel_param_t sel_param = sel_param_t::KP;
float add = 1.0;
float set_val = 0.0;
void set_param()
{
  std::string sel_kp, sel_ki, sel_kd;
  if (M5.BtnB.wasPressed())
  {
    switch (sel_param)
    {
    case sel_param_t::KP:
      sel_param = sel_param_t::KI;
      set_val = pid_param.ki;
      add = 50.0;
      break;
    case sel_param_t::KI:
      sel_param = sel_param_t::KD;
      set_val = pid_param.kd;
      add = 0.5;
      break;
    case sel_param_t::KD:
      sel_param = sel_param_t::KP;

      set_val = pid_param.kp;
      add = 10.0;
      break;
    }
  }
  else
  {
    if (M5.BtnA.wasPressed())
    {
      set_val += add;
    }
    if (M5.BtnC.wasPressed())
    {
      set_val -= add;
      if (set_val < 0)
        set_val = 0;
    }
    switch (sel_param)
    {
    case sel_param_t::KP:
      sel_kp = "*";
      sel_ki = "";
      sel_kd = "";
      pid_param.kp = set_val;
      break;
    case sel_param_t::KI:
      sel_kp = "";
      sel_ki = "*";
      sel_kd = "";
      pid_param.ki = set_val;
      break;
    case sel_param_t::KD:
      sel_kp = "";
      sel_ki = "";
      sel_kd = "*";
      pid_param.kd = set_val;
      break;
    }
    pid.set_parameter(pid_param);
  }
  M5.Display.printf("%skp:%4.0f|%ski:%4.0f|%skd:%2.1f\n", sel_kp.c_str(), pid_param.kp, sel_ki.c_str(), pid_param.ki, sel_kd.c_str(), pid_param.kd);
}
