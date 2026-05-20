#pragma once

#include "config.hpp"

#include <chrono>

namespace sonocontrol {

class PIDController {
public:
    PIDController(double kp = 0.8, double ki = 0.05, double kd = 0.2);
    void reset();
    double compute(double setpoint, double measured);

private:
    double kp_ = 0.8;
    double ki_ = 0.05;
    double kd_ = 0.2;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool has_prev_time_ = false;
    std::chrono::steady_clock::time_point prev_time_{};
    double integral_limit_ = 10.0;
};

ActiveParams apply_pid_to_params(const ActiveParams& base,
                                 double demand,
                                 const ActiveParams& initial,
                                 bool use_amplitude,
                                 bool use_duration,
                                 bool use_duty,
                                 bool use_interval);

} // namespace sonocontrol
