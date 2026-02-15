#include "simplePIDController.h"

#include <algorithm>


SimplePIDController::SimplePIDController(
    float kp, float ki, float kd, 
    float integralMax, 
    float outputMax, 
    DerivativeMode dMode) 
    : m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_outputMax(outputMax), 
      m_integralMax(integralMax),
      m_derivativeMode(dMode) {}

float SimplePIDController::evaluate(float measurement, float setpoint, float dt, float feedForwardVal) {
    // Protect against small dt
    constexpr float MIN_DT = 1e-6f;
    dt = std::max(dt, MIN_DT);

    float error = setpoint - measurement;
    
    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_pidState.last = m_derivativeMode == DerivativeMode::Measurement 
            ? measurement : error;
    }
    
    // --- Proportinal ---
    m_pidState.p  = error;
    
    // --- Integral ---
    m_pidState.i += error * dt;
    m_pidState.i = std::clamp(m_pidState.i, -m_integralMax, m_integralMax);
    
    // --- Derivative ---
    if (m_derivativeMode == DerivativeMode::Measurement)
    {
        float rawd = -(measurement - m_pidState.last) / dt;
        m_pidState.last = measurement;
        m_pidState.d = derivativeFilter(rawd, m_pidState.d, dt);
    } 
    else
    {
        float rawd = (error - m_pidState.last) / dt;
        m_pidState.last = error;
        m_pidState.d = derivativeFilter(rawd, m_pidState.d, dt);
    }
    
    // PID output calculation
    // Apply feedforward m_feedForward
    // This is a simple linear feedforward based on the setpoint
	m_pidState.output = (m_pidState.p * m_kp) + 
                        (m_pidState.i * m_ki) + 
                        (m_pidState.d * m_kd) +
                        feedForwardVal;

    m_pidState.output = std::clamp(m_pidState.output, (-m_outputMax), m_outputMax);
	return m_pidState.output;
}

void SimplePIDController::reset() {
	m_pidState.clear();
    m_started = false;
}

void SimplePIDController::resetIntegral()
{
    m_pidState.i = 0.0f;
}

void SimplePIDController::setDerivativeFilterTau(float tau) {
    m_derivativeTau = std::max(tau, 0.0f);
}

float SimplePIDController::derivativeFilter(float current, float previous, float dt)
{
    float alpha = dt / (m_derivativeTau + dt);
    return (alpha * current) + ((1.0f - alpha) * previous);
}