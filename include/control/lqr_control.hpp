#pragma once
#include <cmath>
#include <functional>
#include <limits>
#include <array>

namespace common_lib
{

  template <typename FLOAT>
  class LQRControl
  {
    private:
      std::array<FLOAT,4> k_;
    public:
      LQRControl(void){}
      LQRControl(std::array<FLOAT,4> k){ set_gain(k); }
      void set_gain(const std::array<FLOAT,4> &k) { k_ = k; }
      FLOAT control(const std::array<FLOAT,4> &state){
        FLOAT u;
        u = k_[0]*state[0] + k_[1]*state[1] + k_[2]*state[2] + k_[3]*state[3];
        return u;
      }
  };

} // namespace common_lib
