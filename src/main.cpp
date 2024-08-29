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
using namespace common_lib;

goblib::UnifiedButton unifiedButton;
rpy_t est_rpy;
float leg_height = parallel_link::DEFAULT_LEG_HEIGHT;

void up_dxl_pos()
{
  auto [theta1, theta2] = parallel_link::inv_kinematics(0.0, leg_height + 20.0 * constants::mm_to_m);
  set_angle(theta1, theta2);
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
  auto [theta1, theta2] = parallel_link::inv_kinematics(0.0, leg_height);
  set_angle(theta1, theta2);
  // デバック
  m_lw.on(false);
  m_rw.on(false);
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
  if (claib_flag)
  {
    gyro_caliblation();
    return;
  }
  dxl_status_t lw = m_lw.get_status();
  dxl_status_t rw = m_rw.get_status();
  float vl = lw.velocity * constants::RPM_TO_MPS * WHEEL_RADIUS;
  float vr = rw.velocity * constants::RPM_TO_MPS * WHEEL_RADIUS;
  float v = (vl + vr) * 0.5;
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
  float raw_ax, raw_ay, raw_az;
  float raw_gx, raw_gy, raw_gz;
  M5.Imu.getAccel(&raw_ax, &raw_ay, &raw_az);
  M5.Imu.getGyro(&raw_gx, &raw_gy, &raw_gz);
  float gx = (raw_gx - gyro_offset[0]) * DEG_TO_RAD;
  float gy = (raw_gy - gyro_offset[1]) * DEG_TO_RAD;
  float gz = (raw_gz - gyro_offset[2]) * DEG_TO_RAD;
  float ax = raw_ax;
  float ay = raw_ay;
  float az = raw_az;
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
  // M5.Display.printf("offset(%6.4f,%6.4f,%6.4f)\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);
  // M5.Display.printf("aRPY(%5.1f,%5.1f,%5.1f)\n", a_rpy.roll * RAD_TO_DEG, a_rpy.pitch * RAD_TO_DEG, a_rpy.yaw * RAD_TO_DEG);
  // M5.Display.printf("gRPY(%5.1f,%5.1f,%5.1f)\n", g_rpy.roll*RAD_TO_DEG, g_rpy.pitch*RAD_TO_DEG, g_rpy.yaw*RAD_TO_DEG);
  M5.Display.printf("RPY(%5.1f,%5.1f,%5.1f)\n", est_rpy.roll * RAD_TO_DEG, est_rpy.pitch * RAD_TO_DEG, est_rpy.yaw * RAD_TO_DEG);
  // Set Goal Velocity using RPM
  // dist += v * dt;
  float diff_angle = normalize_angle(target - est_rpy.roll);
  if (approx_zero(gx, 0.005))
    gx = 0.f;
  pid.set_dt(dt);
  float error = diff_angle / P_RATIO;
  float rpm = pid.control(error) - Kgx * gx + Kv * v;
  bool stop = false;
  if (std::abs(diff_angle) > HALF_PI)
  {
    rpm = 0;
    pid.reset();
    auto [theta1, theta2] = parallel_link::inv_kinematics(0.0, leg_height);
    set_angle(theta1, theta2);
    stop = true;
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
    stop = true;
  }
  m_lw.move(rpm);
  m_rw.move(rpm);
  M5.Display.printf("diff_angle:%5.1f,gx:%5.1f\n", diff_angle * RAD_TO_DEG, gx);
  M5.Display.printf("target:%5.1f error:%5.1f\n", target * RAD_TO_DEG, error);
  M5.Display.printf("rpm:%5.1f\n", rpm);
  M5.Display.printf("v:%5.3f[m/s],", v);
  M5.Display.printf("dd:%5.3f\n", v * dt);
  // 水平維持
  if (!stop)
  {
    float theta_T1 = lpf_x.filtering(diff_angle);
    float theta_T2 = normalize_angle(target_pitch - est_rpy.pitch);
    // float theta_T2 = lpf_y.filtering(normalize_angle(target_pitch - est_rpy.pitch));
    // float tan_theta_T2 = parallel_link::HALF_LEG_WIDTH * std::tan(theta_T2);
    float tan_theta_T2 = 0.0;
    // float xt = -leg_height * std::tan(theta_T1) + v * dt;
    float xt = - v * dt * 0.1;
    float l_yt = leg_height - tan_theta_T2;
    float r_yt = leg_height + tan_theta_T2;
    M5.Display.printf("xt:%5.3f|l:%5.3f|r:%5.3f\n", xt, l_yt, r_yt);
    auto [lf_theta, lr_theta] = parallel_link::inv_kinematics(xt, l_yt);
    auto [rf_theta, rr_theta] = parallel_link::inv_kinematics(xt, r_yt);
    set_angle(lf_theta, rf_theta, lr_theta, rr_theta);
  }
  else
  {
    auto [theta1, theta2] = parallel_link::inv_kinematics(0.0, leg_height);
    set_angle(theta1, theta2);
  }
  // パラメータ設定
  set_param();
  M5.Display.endWrite();
  unifiedButton.draw();
  // delay(1);
}
