#include "simplePIDController.h"

#include <algorithm>


SimplePIDController::SimplePIDController(const PIDConfig& config) 
    : m_config(config) {}

float SimplePIDController::evaluate(float measurement, float setpoint, float dt, float feedForwardVal) {
    // Protect against small dt
    constexpr float MIN_DT = 1e-6f;
    dt = std::max(dt, MIN_DT);

    float error = setpoint - measurement;
    
    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_lastMeasurement = m_config.derivativeMode == DerivativeMode::Measurement 
            ? measurement : error;
    }
    
    // --- Proportinal ---
    m_term.p  = error;
    
    // --- Integral ---
    m_term.i += error * dt;
    m_term.i = std::clamp(m_term.i, -m_config.integralLimit, m_config.integralLimit);
    
    // --- Derivative ---
    float rawDError = 0.0f;
    if (m_config.derivativeMode == DerivativeMode::Measurement)
    {
        rawDError = -(measurement - m_lastMeasurement) / dt;
        m_lastMeasurement = measurement;
    } 
    else
    {
        rawDError = (error - m_lastMeasurement) / dt;
        m_lastMeasurement = error;
    }
    m_term.d = derivativeFilter(rawDError, m_term.d, dt);
    
    // PID output calculation
    // Apply feedforward m_feedForward
    // This is a simple linear feedforward based on the setpoint
	m_term.output = (m_term.p * m_config.gains.kp) + 
                        (m_term.i * m_config.gains.ki) + 
                        (m_term.d * m_config.gains.kd) +
                        feedForwardVal;

    m_term.output = std::clamp(m_term.output, (-m_config.outputLimit), m_config.outputLimit);
	return m_term.output;
}

void SimplePIDController::reset() {
	m_term.p = 0.0f;
	m_term.i = 0.0f;
	m_term.d = 0.0f;
	m_term.output = 0.0f;
    m_lastMeasurement = 0.0f;
    m_started = false;
}

void SimplePIDController::resetIntegral()
{
    m_term.i = 0.0f;
}

float SimplePIDController::derivativeFilter(float current, float previous, float dt)
{
    float alpha = dt / (m_config.derivativeFilterTau + dt);
    return (alpha * current) + ((1.0f - alpha) * previous);
}