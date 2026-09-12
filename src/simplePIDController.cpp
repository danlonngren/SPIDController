#include "simplePIDController.h"

#include <algorithm>

SimplePIDController::SimplePIDController(float output_limit, PIDGains gains, GetTime get_time)
    : m_gains(gains), m_time_func(get_time) {
    // Configure defaults
    setOutputLimit(output_limit);
}

float SimplePIDController::evaluate(float input, float setpoint, float feed_forward_val) {
    // Protect against small dt
    const float current_time = getTimeSafe();
    const float dt = std::max(current_time - m_term.last_dt, m_min_dt);

    float error = setpoint - input;

    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_term.last_input = m_config.derivative_mode == DerivativeMode::Measurement ? input : error;
    }

    // --- Proportinal ---
    m_term.p = error;

    // --- Integral ---
    m_term.i += error * dt;
    if (m_config.integral_limit != 0) {
        m_term.i = std::clamp(m_term.i, -m_config.integral_limit, m_config.integral_limit);
    }

    // --- Derivative ---
    float raw_d_error = 0.0f;
    if (m_config.derivative_mode == DerivativeMode::Measurement) {
        raw_d_error = -(input - m_term.last_input) / dt;
    } else {
        raw_d_error = (error - m_term.last_input) / dt;
    }
    m_term.d = derivativeFilter(raw_d_error, m_term.d, dt);

    // PID output calculation
    // Apply feedforward m_feedForward
    // This is a simple linear feedforward based on the setpoint
    m_term.output = (m_term.p * m_gains.kp) + (m_term.i * m_gains.ki) + (m_term.d * m_gains.kd) +
                    feed_forward_val;

    m_term.output = std::clamp(m_term.output, (-m_config.output_limit), m_config.output_limit);

    m_term.last_input = input;
    m_term.last_error = error;
    m_term.last_dt = current_time;

    return m_term.output;
}

void SimplePIDController::reset() {
    m_term.p = 0.0f;
    m_term.i = 0.0f;
    m_term.d = 0.0f;
    m_term.output = 0.0f;
    m_term.last_input = 0.0f;
    m_started = false;
}

void SimplePIDController::resetIntegral() {
    m_term.i = 0.0f;
}

float SimplePIDController::derivativeFilter(float current, float previous, float dt) const {
    float alpha = dt / (m_config.derivative_filter_tau + dt);
    return (alpha * current) + ((1.0f - alpha) * previous);
}

float SimplePIDController::getTimeSafe() {
    float dt{};
    if (m_time_func != nullptr) {
        dt = std::max(m_time_func(), m_min_dt);
    } else {
        dt = m_min_dt;
    }
    return dt;
}
