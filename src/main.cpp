#include <Arduino.h>
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>
#include <gob_unifiedButton.hpp>
// config
#include "config/config.hpp"
// utility
#include "utility/math_util.hpp"
#include "utility/dynamixel_utils.hpp"
// control
#include "control/pid_control.hpp"
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"
using namespace common_lib;

goblib::UnifiedButton unifiedButton;

PidControl pid;
pid_parameter_t pid_param;
float target;

enum class sel_param_t
{
  KP,
  KI,
  KD
};
sel_param_t sel_param = sel_param_t::KP;
float add = 1.0;
float set_val = 0.0;

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

uint32_t timer;
LowpassFilterf lpf_acc_x(LPF_ALPHA);
LowpassFilterf lpf_acc_y(LPF_ALPHA);
ComplementaryFilterf comp_filter_x(COMP_ALPHA);
ComplementaryFilterf comp_filter_y(COMP_ALPHA);

struct rpy_t
{
  float roll;
  float pitch;
  float yaw;
};
rpy_t est_rpy;
rpy_t acc_rpy(float accX, float accY, float accZ)
{
  rpy_t rpy;
  float Aroll = atan2f(accY, accZ);
  float Apitch = -atan2f(accX, sqrtf(accY * accY + accZ * accZ));
  rpy.roll = Aroll;
  rpy.pitch = Apitch;
  rpy.yaw = 0.0F;
  return rpy;
}

rpy_t gyro_rpy(rpy_t pre, float gyroX, float gyroY, float gyroZ, float dt)
{
  pre.roll += gyroX * dt;
  pre.pitch += gyroY * dt;
  pre.yaw += gyroZ * dt;
  return pre;
}

void test_dxl_pos()
{
  m_lf.move(HALF_PI);
  m_rf.move(HALF_PI);
  m_lr.move(HALF_PI);
  m_rr.move(HALF_PI);
}

void set_dxl_init_pos()
{
  m_lf.move(0.0);
  m_rf.move(0.0);
  m_lr.move(0.0);
  m_rr.move(0.0);
}

void set_angle(float roll, float pitch)
{
  float bias = 0.0;
  m_lf.move(roll + pitch * bias);
  m_rf.move(roll - pitch * bias);
  m_lr.move(-roll + pitch * bias);
  m_rr.move(-roll - pitch * bias);
}

void setup()
{
  // M5設定
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 0);
  // button設定
  unifiedButton.begin(&M5.Display);
  auto btnA = unifiedButton.getButtonA();
  assert(btnA);
  btnA->setLabelText("<");
  auto btnB = unifiedButton.getButtonB();
  assert(btnB);
  btnB->setLabelText("sel");
  auto btnC = unifiedButton.getButtonC();
  assert(btnC);
  btnC->setLabelText(">");
  // dynamixel 設定
  DXL_SERIAL.begin(57600, SERIAL_8N1, RX_SERVO, TX_SERVO);
  dxl = Dynamixel2Arduino(DXL_SERIAL);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // motor setup
  // wheel
  m_lw.begin(true, false);
  m_rw.begin(false, false);
  // leg
  m_lf.begin(false, true);
  m_lr.begin(true, true);
  m_rf.begin(true, true);
  m_rr.begin(false, true);
  // limit
  m_lf.set_limit(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  m_rf.set_limit(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  m_lr.set_limit(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  m_rr.set_limit(MIN_ANGLE_LIMIT, MAX_ANGLE_LIMIT);
  // 初期位置
  set_dxl_init_pos();
  // デバック
  // m_lw.on(false);
  // m_rw.on(false);
  // m_lf.on(false);
  // m_rf.on(false);
  // m_lr.on(false);
  // m_rr.on(false);
  // pid
  pid_param.kp = 600.0;
  pid_param.ki = 2000.0;
  pid_param.kd = 0.0;
  pid_param.control_freq = 1000.0;
  pid_param.output_upper_limit = MAX_RPM;
  pid.set_parameter(pid_param);
  set_val = pid_param.kp;
  target = 0.0;
  timer = micros();
}

void loop()
{
  M5.update();
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  unifiedButton.update();
  M5.Display.startWrite();
  M5.Display.setCursor(0, 0);
  bool pressed = false;
  int x = 0, y = 0;
  if (M5.Touch.isEnabled())
  {
    auto t = M5.Touch.getDetail();
    x = t.distanceX();
    y = t.distanceY();
    pressed = t.isPressed();
  }
  M5.Display.printf("x:%4d y:%4d press:%d,%4.3f\n", x, y, pressed,dt);
  // IMU
  float ax, ay, az;
  float gx, gy, gz;
  M5.Imu.getAccel(&ax, &ay, &az);
  M5.Imu.getGyro(&gx, &gy, &gz);
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;
  // 姿勢計算
  rpy_t a_rpy = acc_rpy(ax, ay, az);
  rpy_t g_rpy = gyro_rpy(est_rpy, gx, gy, gz, dt);
  est_rpy.yaw = g_rpy.yaw;
  a_rpy.roll = normalize_angle(a_rpy.roll - HALF_PI);
  // a_rpy.roll = lpf_acc_x.filtering(a_rpy.roll);
  // a_rpy.pitch = lpf_acc_y.filtering(a_rpy.pitch);
  // est_rpy.roll = a_rpy.roll;
  // est_rpy.pitch = a_rpy.pitch;
  est_rpy.roll = comp_filter_x.filtering(a_rpy.roll, g_rpy.roll);
  est_rpy.pitch = comp_filter_y.filtering(a_rpy.pitch, g_rpy.pitch);
  M5.Display.printf("acc(%5.1f,%5.1f,%5.1f)\n", ax, ay, az);
  M5.Display.printf("gyro(%5.1f,%5.1f,%5.1f)\n", gx, gy, gz);
  M5.Display.printf("aRPY(%5.1f,%5.1f,%5.1f)\n", a_rpy.roll, a_rpy.pitch, a_rpy.yaw);
  M5.Display.printf("gRPY(%5.1f,%5.1f,%5.1f)\n", g_rpy.roll, g_rpy.pitch, g_rpy.yaw);
  M5.Display.printf("RPY(%5.1f,%5.1f,%5.1f)\n", est_rpy.roll, est_rpy.pitch, est_rpy.yaw);
  // Set Goal Velocity using RPM
  pid.set_dt(dt);
  float error = normalize_angle(target - est_rpy.roll);
  float rpm = pid.control(error);
  if (pressed)
  {
    // target = est_rpy.roll;
    rpm = 0;
    // test_dxl_pos();
    set_dxl_init_pos();
  }
  // else
  //   set_angle(est_rpy.roll,est_rpy.pitch);
  if (std::abs(est_rpy.roll) > HALF_PI)
  {
    rpm = 0;
  }
  m_lw.move(rpm);
  m_rw.move(rpm);
  M5.Display.printf("target:%5.1f error:%5.1f\n", target, error);
  M5.Display.printf("rpm:%5.1f\n", rpm);
  // reg state
  // dxl_status_t lw = m_lw.get_status();
  // dxl_status_t rw = m_rw.get_status();
  dxl_status_t lf = m_lf.get_status();
  dxl_status_t rf = m_rf.get_status();
  dxl_status_t lr = m_lr.get_status();
  dxl_status_t rr = m_rr.get_status();
  M5.Display.printf("LF: %5.1f LR:  %5.1f\n", lf.position * RAD_TO_DEG, lr.position * RAD_TO_DEG);
  M5.Display.printf("RF: %5.1f RR:  %5.1f\n", rf.position * RAD_TO_DEG, rr.position * RAD_TO_DEG);
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
  M5.Display.endWrite();
  unifiedButton.draw();
  // delay(1);
}
