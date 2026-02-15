#include "simplePIDController.h"

#include <algorithm>


SimplePIDController::SimplePIDController(
    float kp, float ki, float kd, 
    float integralMax, 
    float outMax, 
    DerivativeMode dMode) 
    : m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_maxOutput(outMax), 
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
        m_pidData.eLast = m_derivativeMode == DerivativeMode::Measurement 
            ? measurement : error;
    }
    
    // --- Proportinal ---
    m_pidData.pError  = error;
    
    // --- Integral ---
    m_pidData.iError += error * dt;
    m_pidData.iError = std::clamp(m_pidData.iError, -m_integralMax, m_integralMax);
    
    // --- Derivative ---
    if (m_derivativeMode == DerivativeMode::Measurement)
    {
        float rawDError = -(measurement - m_pidData.eLast) / dt;
        m_pidData.eLast = measurement;
        m_pidData.dError = derivativeFilter(rawDError, m_pidData.dError, dt);
    } 
    else
    {
        float rawDError = (error - m_pidData.eLast) / dt;
        m_pidData.eLast = error;
        m_pidData.dError = derivativeFilter(rawDError, m_pidData.dError, dt);
    }
    
    // PID output calculation
    // Apply feedforward m_feedForward
    // This is a simple linear feedforward based on the setpoint
	m_pidOutput = (m_pidData.pError * m_kp) + 
                  (m_pidData.iError * m_ki) + 
                  (m_pidData.dError * m_kd) +
                  feedForwardVal;

    m_pidOutput = std::clamp(m_pidOutput, (-m_maxOutput), m_maxOutput);
	return m_pidOutput;
}

void SimplePIDController::reset() {
	m_pidData.clear();
    m_pidOutput = 0.0f;
    m_started = false;
}

void SimplePIDController::resetIntegral()
{
    m_pidData.iError = 0.0f;
}

void SimplePIDController::setDerivativeFilterTau(float tau) {
    m_derivativeTau = std::max(tau, 0.0f);
}

float SimplePIDController::derivativeFilter(float current, float previous, float dt)
{
    float alpha = dt / (m_derivativeTau + dt);
    return (alpha * current) + ((1.0f - alpha) * previous);
}