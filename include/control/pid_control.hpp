#pragma once
#include <cmath>
#include <functional>
#include <limits>

namespace common_lib {

  struct pid_parameter_t {
    double control_freq         = 1000;
    double kp                   = 0;
    double ki                   = 0;
    double kd                   = 0;
    double lpf_time_constant    = 0;
    double output_upper_limit   = 0;
    double integral_upper_limit = 0;
  };

  class PidControl {
    pid_parameter_t param_;
    double integral_error_;
    double before_target_;
    double before_error_;
    double dt_;
    double lpf_gain_;
    double ff_slope_, ff_intercept_, ff_approx_zero_; // ff用の値
    std::function<double(double, double)> ff_function_;

  public:
    PidControl(void);
    PidControl(pid_parameter_t param);
    void set_parameter(pid_parameter_t param);
    void set_ff_function(std::function<double(double, double)> f);
    void set_ff_function(double slope, double intercept, double near_zero = 1e-12);
    pid_parameter_t get_parameter(void) const { return param_; }
    pid_parameter_t* get_parameter_ptr(void) { return &param_; }
    void set_dt(double dt) {
      dt_       = dt;
      lpf_gain_ = dt_ / (dt_ + param_.lpf_time_constant);
    }
    double control(double target, double present);
    double control(double error);
    void reset(void);

    static double calc_kp(double time_const);
  };

  PidControl::PidControl(void) : ff_slope_(0), ff_intercept_(0), ff_approx_zero_(0) {
    pid_parameter_t param;
    param_          = param;
    integral_error_ = 0;
    before_target_  = 0;
    before_error_   = 0;
    dt_             = 1.0 / param_.control_freq;
    lpf_gain_       = dt_ / (dt_ + param_.lpf_time_constant);
    ff_function_    = [](double target, double present) { return 0.0; };
  }

  PidControl::PidControl(pid_parameter_t param) : ff_slope_(0), ff_intercept_(0), ff_approx_zero_(0) {
    param_          = param;
    integral_error_ = 0;
    before_target_  = 0;
    dt_             = 1.0 / param_.control_freq;
    lpf_gain_       = dt_ / (dt_ + param_.lpf_time_constant);
    ff_function_    = [](double target, double present) { return 0.0; };
  }

  void PidControl::set_parameter(pid_parameter_t param) {
    param_    = param;
    dt_       = 1.0 / param_.control_freq;
    lpf_gain_ = dt_ / (dt_ + param_.lpf_time_constant);
  }

  void PidControl::set_ff_function(std::function<double(double, double)> func) {
    ff_function_ = func; // set_parameterより後に実行する
  }

  void PidControl::set_ff_function(double slope, double intercept, double approx_zero) {
    ff_slope_ = slope, ff_intercept_ = intercept, ff_approx_zero_ = approx_zero;
    ff_function_ = [this](double target, double present) {
      if (abs(target) > ff_approx_zero_) {
        return target * ff_slope_ + ((target > 0) ? ff_intercept_ : -ff_intercept_);
      }
      return 0.0;
    };
  }

  double PidControl::control(double target, double present) {
    double error = target - present;
    double ret   = 0;

    ret += ff_function_(target, present); // FF

    ret += (error * param_.kp); // P

    integral_error_ += error * dt_;
    if (integral_error_ > param_.integral_upper_limit) {
      integral_error_ = param_.integral_upper_limit;
    } else if (integral_error_ < -param_.integral_upper_limit) {
      integral_error_ = -param_.integral_upper_limit;
    }
    ret += (integral_error_ * param_.ki); // I

    double lpf_error = before_error_ + lpf_gain_ * (error - before_error_);
    before_error_    = lpf_error;
    ret += ((lpf_error - before_error_) / dt_) * param_.kd; // D

    if (ret > param_.output_upper_limit) ret = param_.output_upper_limit;
    if (ret < -param_.output_upper_limit) ret = -param_.output_upper_limit;
    return ret;
  }

  double PidControl::control(double error) {
    double ret = 0;

    // ret += ff_function_(error, 0);	// FF

    ret += (error * param_.kp); // P

    integral_error_ += error * dt_;
    if (integral_error_ > param_.integral_upper_limit) {
      integral_error_ = param_.integral_upper_limit;
    } else if (integral_error_ < -param_.integral_upper_limit) {
      integral_error_ = -param_.integral_upper_limit;
    }
    ret += (integral_error_ * param_.ki); // I

    double lpf_error = before_error_ + lpf_gain_ * (error - before_error_);
    before_error_    = lpf_error;
    ret += ((lpf_error - before_error_) / dt_) * param_.kd; // D

    if (ret > param_.output_upper_limit) ret = param_.output_upper_limit;
    if (ret < -param_.output_upper_limit) ret = -param_.output_upper_limit;
    return ret;
  }

  void PidControl::reset(void) {
    integral_error_ = 0;
    before_target_  = 0;
    before_error_   = 0;
  }

  double PidControl::calc_kp(double time_constant) { return 1.0 / time_constant; }

} // namespace common_lib