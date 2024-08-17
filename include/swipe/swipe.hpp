#pragma once
#include <Arduino.h>
#include <M5Unified.h>
#include "utility/math_util.hpp"

namespace common_lib
{
class Swipe
{
private:
  int last_x_, last_y_;
  int x_, y_;
  bool pressed_;
  bool set_last_ = false;

  float swipe_x_, swipe_y_;
public:
  Swipe() {}
  Swipe(int swipe_x, int swipe_y) : swipe_x_(swipe_x), swipe_y_(swipe_y) {}
  bool isSwipe()
  {
    if (pressed_)
    {
      int diff_x = x_ - last_x_;
      int diff_y = y_ - last_y_;
      if (std::abs(diff_x) > swipe_x_ && swipe_x_ != 0)
      {
        if(sign(diff_x) == sign(swipe_x_))
          return true;
      }
      if (std::abs(diff_y) > swipe_y_ && swipe_y_ != 0)
      {
        if(sign(diff_y) == sign(swipe_y_))
          return true;
      }
    }
    return false;
  }
  void update(int x, int y, bool pressed)
  {
    if (pressed && !set_last_)
    {
      last_x_ = x;
      last_y_ = y;
      set_last_ = true;
    }
    if (!pressed && set_last_)
    {
      set_last_ = false;
    }
    x_ = x;
    y_ = y;
    pressed_ = pressed;
  }
};
} // namespace common_lib
