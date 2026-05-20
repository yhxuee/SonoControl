#include "pid.hpp"

#include <algorithm>

namespace sonocontrol {

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    has_prev_time_ = false;
}

double PIDController::compute(double setpoint, double measured) {
    const auto now = std::chrono::steady_clock::now();
    const double error = setpoint - measured;

    double dt = 0.1;
    double derivative = 0.0;
    if (has_prev_time_) {
        dt = std::chrono::duration<double>(now - prev_time_).count();
        dt = std::max(0.001, dt);
        derivative = (error - prev_error_) / dt;
    }
    prev_time_ = now;
    has_prev_time_ = true;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    prev_error_ = error;

    const double raw = kp_ * error + ki_ * integral_ + kd_ * derivative;
    // +0.5 biases the demand to mid-scale (50 % power) when the PID terms sum
    // to zero. Without it, a freshly-reset controller at setpoint would start
    // at demand=0 and only ramp up via integral wind-up, causing a visible
    // temperature dip at cycle boundaries. The bias ensures the system
    // maintains roughly half power as a starting point even before the
    // integral has time to track the true steady-state load.
    return std::clamp(raw + 0.5, 0.0, 1.0);
}

ActiveParams apply_pid_to_params(const ActiveParams& base,
                                 double demand,
                                 const ActiveParams& initial,
                                 bool use_amplitude,
                                 bool use_duration,
                                 bool use_duty,
                                 bool use_interval) {
    ActiveParams result = base;
    demand = std::clamp(demand, 0.0, 1.0);

    if (use_amplitude) {
        result.amplitude = demand * initial.amplitude;
    }
    if (use_duration) {
        result.duration_ms = static_cast<int>(demand * static_cast<double>(initial.duration_ms));
    }
    if (use_duty) {
        result.duty_cycle = demand * initial.duty_cycle;
    }
    if (use_interval) {
        const double factor = 1.0 + (1.0 - demand) * 2.0;
        result.interval_time_s = initial.interval_time_s * factor;
    }

    const double min_interval = static_cast<double>(result.duration_ms) / 1000.0;
    result.interval_time_s = std::max(min_interval, result.interval_time_s);

    result.amplitude = std::min(result.amplitude, initial.amplitude);
    result.duration_ms = std::min(result.duration_ms, initial.duration_ms);
    result.duty_cycle = std::min(result.duty_cycle, initial.duty_cycle);

    return result;
}

} // namespace sonocontrol
