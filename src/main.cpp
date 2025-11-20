#include <SD.h>
#include <Arduino.h>
#include <M5Unified.h>
#include <Dynamixel2Arduino.h>
#include <gob_unifiedButton.hpp>
// ota
#define ESP32_RTOS
#include "ota/ota.h"
// config
#include "config/config.hpp"
// utility
#include "utility/math_util.hpp"
#include "utility/dynamixel_utils.hpp"
#include "utility/imu_util.hpp"
#include "utility/string_util.hpp"
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

float torque_control(float error, float anguler, float motor_velocity, const pid_parameter_t &pid_param)
{
  float torque = pid_param.kp * error + pid_param.kd * anguler + Kw * motor_velocity;
  return torque;
}

const char *fname = "/wifi.csv";
File fp;
std::string ssid;
std::string password;

void setup()
{
  Serial.begin(115200);
  // USBSerial.begin(115200);
  // AW9523(拡張IO)
  // aw9523_begin();
  // bool sdexist = sd_exist();
  bool sdexist = false;
  // M5設定
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.printf("SD CARD:%d\n", sdexist);
  // SD
  if (sdexist)
  {
    while (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000))
    {
      Serial.println("SD CARD ERROR");
      delay(1000);
    }
    // ota設定
    // setupOTA();
    fp = SD.open(fname);
    if (fp)
    {
      unsigned int cnt = 0;
      char data[64];
      char *str;
      bool flag = false;
      Serial.println("file reading");
      while (fp.available())
      {
        data[cnt++] = fp.read();
        flag = true;
      }
      if (flag)
      {
        std::string str_data = std::string(data);
        std::vector<std::string> str_l = split(str_data, "\n");
        if (str_l.size() >= 2)
        {
          std::vector<std::string> vec1 = split(str_l[0], ",");
          std::vector<std::string> vec2 = split(str_l[1], ",");
          if (vec1.size() >= 2 && vec2.size() >= 2)
          {
            if (vec1[0] == "SSID" && vec2[0] == "PASS")
            {
              ssid = vec1[1];
              password = vec2[1];
              Serial.println("wifi info");
              Serial.printf("ssid:%s\n", ssid.c_str());
              Serial.printf("pass:%s\n", password.c_str());
              setupOTA(ssid.c_str(), password.c_str());
            }
          }
        }
      }
      else
        Serial.println("file read error");
      fp.close();
    }
    else
      Serial.println("file open error");
  }
  else
  {
    // ota設定
    setupOTA();
    Serial.println("SD not exist");
  }
  // ディスプレイ設定
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);
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
  pid_param.kd = -1.5;
  pid_param.control_freq = 1000.0;
  pid_param.output_upper_limit = MAX_RPM;
  pid_param.integral_upper_limit = 1000.0;
  // pid.set_parameter(pid_param);
  set_val = pid_param.kp;
  // lqr.set_gain(LQR_K);
  timer = micros();

  Serial.printf("ssid:%s\n", ssid.c_str());
  Serial.printf("ip:%s\n", WiFi.localIP().toString().c_str());
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
  float wheel_v = (vl + vr) * 0.5;
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
  M5.Display.printf("ssid:%s\n", ssid.c_str());
  M5.Display.printf("ip:%s\n", WiFi.localIP().toString().c_str());
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
  // pid.set_dt(dt);
  float error = diff_angle / P_RATIO;
  // float rpm = pid.control(error) - Kgx * gx + Kv * v;
  float torque = torque_control(error, gx, wheel_v, pid_param);
  bool stop = false;
  if (std::abs(diff_angle) > HALF_PI)
  {
    torque = 0;
    // pid.reset();
    auto [theta1, theta2] = parallel_link::inv_kinematics(0.0, leg_height);
    set_angle(theta1, theta2);
    stop = true;
  }
  // swipe
  if (l_swipe.isSwipe())
  {
    torque = 0;
    m_lw.on(true);
    m_rw.on(true);
  }
  else if (r_swipe.isSwipe())
  {
    torque = 0;
    m_lw.on(false);
    m_rw.on(false);
  }
  else if (u_swipe.isSwipe())
  {
    torque = 0;
    // up_dxl_pos();
  }
  else if (d_swipe.isSwipe())
  {
    target = est_rpy.roll;
  }
  else if (!u_swipe.isSwipe() && pressed)
  {
    torque = 0;
    // pid.reset();
    stop = true;
  }
  m_lw.move(torque);
  m_rw.move(torque);
  M5.Display.printf("diff_angle:%5.1f,gx:%5.1f\n", diff_angle * RAD_TO_DEG, gx);
  M5.Display.printf("target:%5.1f error:%5.1f\n", target * RAD_TO_DEG, error);
  M5.Display.printf("torque:%5.1f\n", torque);
  M5.Display.printf("v:%5.3f[m/s]\n,", wheel_v);
  // M5.Display.printf("dd:%5.3f\n", v * dt);
  // 水平維持
  stop = true;
  if (!stop)
  {
    float theta_T1 = lpf_x.filtering(diff_angle);
    float theta_T2 = normalize_angle(target_pitch - est_rpy.pitch);
    // float theta_T2 = lpf_y.filtering(normalize_angle(target_pitch - est_rpy.pitch));
    // float tan_theta_T2 = parallel_link::HALF_LEG_WIDTH * std::tan(theta_T2);
    float tan_theta_T2 = 0.0;
    float xt = -leg_height * std::tan(theta_T1);
    // float xt = - v * dt * 0.1;
    float l_yt = leg_height - tan_theta_T2;
    float r_yt = leg_height + tan_theta_T2;
    M5.Display.printf("xt:%5.3f|l:%5.3f|r:%5.3f\n", xt, l_yt, r_yt);
    auto [lf_theta, lr_theta] = parallel_link::inv_kinematics(xt, l_yt);
    auto [rf_theta, rr_theta] = parallel_link::inv_kinematics(xt, r_yt);
    M5.Display.printf("lf:%5.3f lr:%5.3f\n", lf_theta, lr_theta);
    M5.Display.printf("rf:%5.3f rr:%5.3f\n", rf_theta, rr_theta);
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
