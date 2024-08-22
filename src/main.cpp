#include <Arduino.h>
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>
#include <gob_unifiedButton.hpp>
// config
#include "config/config.hpp"
// utility
#include "utility/math_util.hpp"
#include "utility/dynamixel_utils.hpp"
#include "utility/imu_util.hpp"
// control
#include "control/pid_control.hpp"
#include "control/lqr_control.hpp"
// filter
#include "filter/lowpass_filter.hpp"
#include "filter/complementary_filter.hpp"

#include "swipe/swipe.hpp"
using namespace common_lib;

goblib::UnifiedButton unifiedButton;

uint32_t timer;
rpy_t est_rpy;

void up_dxl_pos()
{
  m_lf.move(HALF_PI);
  m_rf.move(HALF_PI);
  m_lr.move(HALF_PI);
  m_rr.move(HALF_PI);
}

void set_angle(float angle, float roll, float pitch)
{
  float bias = 0.0;
  m_lf.move(angle + roll + pitch * bias);
  m_rf.move(angle + roll - pitch * bias);
  m_lr.move(angle - roll + pitch * bias);
  m_rr.move(angle - roll - pitch * bias);
}

Swipe l_swipe(-50, 0);
Swipe r_swipe(50, 0);
Swipe u_swipe(0, -50);
Swipe d_swipe(0, 30);

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
  set_dxl_pos(DEFAULT_ANGLE);
  // デバック
  // m_lw.on(false);
  // m_rw.on(false);
  // m_lf.on(false);
  // m_rf.on(false);
  // m_lr.on(false);
  // m_rr.on(false);
  // pid
  pid_param.kp = 60.0;
  pid_param.ki = 0.0;
  pid_param.kd = 0.0;
  pid_param.control_freq = 1000.0;
  pid_param.output_upper_limit = MAX_RPM;
  pid_param.integral_upper_limit = 1000.0;
  pid.set_parameter(pid_param);
  set_val = pid_param.kp;
  // lqr.set_gain(LQR_K);
  timer = micros();
}

void loop()
{
  M5.update();
  // dxl_status_t lw = m_lw.get_status();
  // dxl_status_t rw = m_rw.get_status();
  // float vl = lw.velocity*constants::RPM_TO_MPS*WHEEL_RADIUS;
  // float vr = rw.velocity*constants::RPM_TO_MPS*WHEEL_RADIUS;
  // float v = (vl + vr) / 2.0;
  float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
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
    l_swipe.update(x, y, pressed);
    r_swipe.update(x, y, pressed);
    u_swipe.update(x, y, pressed);
    d_swipe.update(x, y, pressed);
  }
  // M5.Display.printf("x:%4d y:%4d press:%d\n", x, y, pressed);
  M5.Display.printf("dt:%4.3f\n", dt);
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
  // a_rpy.roll = normalize_angle(a_rpy.roll - HALF_PI);
  // est_rpy.roll = lpf_acc_x.filtering(a_rpy.roll);
  // est_rpy.pitch = lpf_acc_y.filtering(a_rpy.pitch);
  est_rpy.roll = comp_filter_x.filtering(a_rpy.roll, g_rpy.roll);
  est_rpy.pitch = comp_filter_y.filtering(a_rpy.pitch, g_rpy.pitch);
  // M5.Display.printf("acc(%5.1f,%5.1f,%5.1f)\n", ax, ay, az);
  // M5.Display.printf("gyro(%5.1f,%5.1f,%5.1f)\n", gx, gy, gz);
  M5.Display.printf("aRPY(%5.1f,%5.1f,%5.1f)\n", a_rpy.roll * RAD_TO_DEG, a_rpy.pitch * RAD_TO_DEG, a_rpy.yaw * RAD_TO_DEG);
  // M5.Display.printf("gRPY(%5.1f,%5.1f,%5.1f)\n", g_rpy.roll*RAD_TO_DEG, g_rpy.pitch*RAD_TO_DEG, g_rpy.yaw*RAD_TO_DEG);
  M5.Display.printf("RPY(%5.1f,%5.1f,%5.1f)\n", est_rpy.roll * RAD_TO_DEG, est_rpy.pitch * RAD_TO_DEG, est_rpy.yaw * RAD_TO_DEG);
  // Set Goal Velocity using RPM
  // dist += v * dt;
  float diff_angle = normalize_angle(target - est_rpy.roll);
  gx -= 0.0011;
  if (approx_zero(gx, 0.005))
    gx = 0.f;
  M5.Display.printf("%5.3f,%5.3f\n", diff_angle, gx);
  pid.set_dt(dt);
  float error = diff_angle / P_RATIO;
  float rpm = pid.control(error - Kgx * gx);
  if (std::abs(diff_angle) > HALF_PI)
  {
    rpm = 0;
    pid.reset();
  }
  // swipe
  if (l_swipe.isSwipe())
  {
    rpm = 0;
    m_lw.on(true);
    m_rw.on(true);
  }
  else if (r_swipe.isSwipe())
  {
    rpm = 0;
    m_lw.on(false);
    m_rw.on(false);
  }
  else if (u_swipe.isSwipe())
  {
    rpm = 0;
    // up_dxl_pos();
  }
  else if (d_swipe.isSwipe())
  {
    target = est_rpy.roll;
  }
  else if (!u_swipe.isSwipe() && pressed)
  {
    rpm = 0;
    pid.reset();
    set_dxl_pos(DEFAULT_ANGLE);
  }
  set_angle(DEFAULT_ANGLE, -2.0 * lpf_roll.filtering(diff_angle), est_rpy.pitch);
  m_lw.move(rpm);
  m_rw.move(rpm);
  M5.Display.printf("target:%5.1f error:%5.1f\n", target * RAD_TO_DEG, error);
  M5.Display.printf("rpm:%5.1f\n", rpm);
  // reg state
  // dxl_status_t lw = m_lw.get_status();
  // dxl_status_t rw = m_rw.get_status();
  // dxl_status_t lf = m_lf.get_status();
  // dxl_status_t rf = m_rf.get_status();
  // dxl_status_t lr = m_lr.get_status();
  // dxl_status_t rr = m_rr.get_status();
  // M5.Display.printf("LF: %5.1f LR:  %5.1f\n", lf.position * RAD_TO_DEG, lr.position * RAD_TO_DEG);
  // M5.Display.printf("RF: %5.1f RR:  %5.1f\n", rf.position * RAD_TO_DEG, rr.position * RAD_TO_DEG);
  set_param();
  M5.Display.endWrite();
  unifiedButton.draw();
  // delay(1);
}
