#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/dynamixel_utils.hpp"
#include <tuple>
// math
#include "utility/math_util.hpp"
// control
#include "control/pid_control.hpp"
#include "control/lqr_control.hpp"
// filter
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"
// swipe
#include "swipe/swipe.hpp"
// kinematics
#include "parallel_link/parallel_link.hpp"

#define DEBUG_SERIAL Serial
HardwareSerial &DXL_SERIAL = Serial1;

// M5Stack Core2 PORT A
constexpr uint8_t RX_SERVO = 9; // 32
constexpr uint8_t TX_SERVO = 8; // 33

// 右 id
constexpr uint8_t DXL_ID_LW = 1;
constexpr uint8_t DXL_ID_LF = 3;
constexpr uint8_t DXL_ID_LR = 4;
// 左 id
constexpr uint8_t DXL_ID_RW = 2;
constexpr uint8_t DXL_ID_RF = 5;
constexpr uint8_t DXL_ID_RR = 6;

constexpr float DXL_PROTOCOL_VERSION = 2.0;

// constexpr float MAX_RPM = 101.0;// M288
constexpr float MAX_RPM = 370.0; // M077
constexpr float MIN_ANGLE_LIMIT = -10.0 * DEG_TO_RAD;
constexpr float MAX_ANGLE_LIMIT = 90.0 * DEG_TO_RAD;

constexpr float WHEEL_RADIUS = 0.0285; // [m]

constexpr float LPF_ALPHA = 0.88;
constexpr float COMP_ALPHA = 0.95;

constexpr float P_RATIO = 0.09;

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

float target = 95.0 * DEG_TO_RAD;
float target_pitch = 0.0 * DEG_TO_RAD;
// set dxl pos
constexpr float DEFAULT_ANGLE = 14.0 * DEG_TO_RAD;
void set_dxl_pos(double angle = 0.0)
{
  m_lf.move(angle);
  m_rf.move(angle);
  m_lr.move(angle);
  m_rr.move(angle);
}

void set_angle(float f_angle, float r_angle)
{
  m_lf.move(f_angle);
  m_rf.move(f_angle);
  m_lr.move(r_angle);
  m_rr.move(r_angle);
}

void set_angle(float lf, float rf, float lr, float rr)
{
  m_lf.move(lf);
  m_rf.move(rf);
  m_lr.move(lr);
  m_rr.move(rr);
}

// gyro
constexpr float CALIB_TIME = 2.0;
bool claib_flag = false;
int calib_count = 0;
uint32_t timer;
std::array<float, 3> gyro_offset = {-0.0014, 0.0045, 0.0};
void gyro_caliblation()
{
  float calib_time = (float)(micros() - timer) / 1000000;
  if (calib_time > CALIB_TIME)
    claib_flag = false;
  M5.Display.startWrite();
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Gyro Calibration\n");
  M5.Display.printf("time:%4.3f\n", calib_time);
  float gx, gy, gz;
  M5.Imu.getGyro(&gx, &gy, &gz);
  M5.Display.printf("gyro(%5.1f,%5.1f,%5.1f)\n", gx, gy, gz);
  gyro_offset[0] += gx;
  gyro_offset[1] += gy;
  gyro_offset[2] += gz;
  calib_count++;
  gyro_offset[0] /= calib_count;
  gyro_offset[1] /= calib_count;
  gyro_offset[2] /= calib_count;
  // M5.Display.printf("offset(%5.4f,%5.4f,%5.4f)\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  M5.Display.printf("count:%d\n", calib_count);
  M5.Display.endWrite();
}

// filter
common_lib::LowpassFilterf lpf_x(0.7); //0.09
common_lib::LowpassFilterf lpf_y(0.95);
common_lib::LowpassFilterf lpf_acc_x(LPF_ALPHA);
common_lib::LowpassFilterf lpf_acc_y(LPF_ALPHA);
common_lib::ComplementaryFilterf comp_filter_x(COMP_ALPHA);
common_lib::ComplementaryFilterf comp_filter_y(COMP_ALPHA);

// lqr control
// common_lib::LQRControl<float> lqr;
// // std::array<float,4> state;
// const std::array<float, 4> LQR_K = {-1.0,20.86242648,-2.06270475,3.26909677};

// pid parameter setting
common_lib::PidControl pid;
common_lib::pid_parameter_t pid_param;
enum class sel_param_t
{
  KP,
  KI,
  KD,
  KGX,
  KV
};
sel_param_t sel_param = sel_param_t::KP;
float Kgx = 20.0;
float Kv = -0.0;
float add = 1.0;
float set_val = 0.0;
void set_param()
{
  std::string sel_kp, sel_ki, sel_kd, sel_kgx, sel_kv;
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
      sel_param = sel_param_t::KGX;

      set_val = Kgx;
      add = 0.1;
      break;
    case sel_param_t::KGX:
      sel_param = sel_param_t::KV;
      set_val = Kv;
      add = 0.1;
      break;
    case sel_param_t::KV:
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
      if (set_val < 0 &&  sel_param != sel_param_t::KV)
        set_val = 0;
    }
    switch (sel_param)
    {
    case sel_param_t::KP:
      sel_kp = "*";
      sel_ki = "";
      sel_kd = "";
      sel_kgx = "";
      sel_kv = "";
      pid_param.kp = set_val;
      break;
    case sel_param_t::KI:
      sel_kp = "";
      sel_ki = "*";
      sel_kd = "";
      sel_kgx = "";
      sel_kv = "";
      pid_param.ki = set_val;
      break;
    case sel_param_t::KD:
      sel_kp = "";
      sel_ki = "";
      sel_kd = "*";
      sel_kgx = "";
      sel_kv = "";
      pid_param.kd = set_val;
      break;
    case sel_param_t::KGX:
      sel_kp = "";
      sel_ki = "";
      sel_kd = "";
      sel_kgx = "*";
      sel_kv = "";
      Kgx = set_val;
      break;
    case sel_param_t::KV:
      sel_kp = "";
      sel_ki = "";
      sel_kd = "";
      sel_kgx = "";
      sel_kv = "*";
      Kv = set_val;
      break;
    }
    pid.set_parameter(pid_param);
  }
  M5.Display.printf("%skp:%4.0f|%ski:%4.0f|%skd:%2.1f\n", sel_kp.c_str(), pid_param.kp, sel_ki.c_str(), pid_param.ki, sel_kd.c_str(), pid_param.kd);
  M5.Display.printf("%sKgx:%2.1f|%sKv:%2.1f\n", sel_kgx.c_str(), Kgx, sel_kv.c_str(), Kv);
}

// swipe
common_lib::Swipe l_swipe(-50, 0);
common_lib::Swipe r_swipe(50, 0);
common_lib::Swipe u_swipe(0, -50);
common_lib::Swipe d_swipe(0, 30);
