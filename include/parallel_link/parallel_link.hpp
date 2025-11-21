#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/dynamixel_utils.hpp"
#include <tuple>
// math
#include "utility/math_util.hpp"

namespace parallel_link
{
  // パラレルリンクパラメータ
  constexpr float l1 = 30.0 * common_lib::constants::mm_to_m;  // 短いリンクの長さ
  constexpr float l2 = 70.0 * common_lib::constants::mm_to_m;  // 長いリンクの長さ
  constexpr float xm1 = 32.0 * common_lib::constants::mm_to_m; // ロボット中心からモーターの回転軸までの位置
  constexpr float ym1 = 0.0;
  // 足の間隔
  constexpr float LEG_WIDTH = 160.0 * common_lib::constants::mm_to_m;
  constexpr float HALF_LEG_WIDTH = LEG_WIDTH / 2.0;
  // 制限
  // 足の高さ 35 ~ 90mm
  constexpr float MIN_LEG_HEIGHT = 35.0 * common_lib::constants::mm_to_m;
  constexpr float MAX_LEG_HEIGHT = 90.0 * common_lib::constants::mm_to_m;
  // constexpr float DEFAULT_LEG_HEIGHT = 40.0 * common_lib::constants::mm_to_m;
  constexpr float DEFAULT_LEG_HEIGHT = 35.0 * common_lib::constants::mm_to_m;

  std::tuple<double, double> inv_kinematics(float xT, float yT)
  {
    using namespace common_lib;
    if (yT < MIN_LEG_HEIGHT)
      yT = MIN_LEG_HEIGHT;
    if (yT > MAX_LEG_HEIGHT)
      yT = MAX_LEG_HEIGHT;
    static float theta1 = 0.0;
    static float theta2 = 0.0;

    constexpr float l1_2 = l1 * l1;
    constexpr float l2_2 = l2 * l2;
    // 前側リンク
    float dx = xT + xm1;
    float dy = yT - ym1;
    float c = std::sqrt(dx * dx + dy * dy);
    float theta_a = std::atan2(dy, dx);
    float theta_b = std::acos((l1_2 + c * c - l2_2) / (2 * l1 * c));
    float thet_ab = normalize_angle(theta_a + theta_b);
    float theta1_ = normalize_angle(PI - thet_ab);
    if (!std::isnan(theta1_) && !std::isinf(theta1_))
      theta1 = theta1_;
    // 後側リンク
    dx = xm1 - xT;
    c = std::sqrt(dx * dx + dy * dy);
    theta_a = std::atan2(dy, dx);
    theta_b = std::acos((l1_2 + c * c - l2_2) / (2 * l1 * c));
    thet_ab = normalize_angle(theta_a + theta_b);
    float theta2_ = normalize_angle(PI - thet_ab);
    if (!std::isnan(theta2_) && !std::isinf(theta2_))
      theta2 = theta2_;
    // M5.Display.printf("theta:%5.1f,%5.1f\n", theta1 * RAD_TO_DEG, theta2 * RAD_TO_DEG);
    return std::make_tuple(theta1, theta2);
  }

} // namespace parallel_link
