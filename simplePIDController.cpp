#include "simplePIDController.h"

#include <algorithm>


SimplePIDController::SimplePIDController(float integralMax, float outMax) :
    m_maxOutput(outMax), 
    m_integralMax(integralMax) {}

float SimplePIDController::evaluate(float error, float dt, float feedForwardVal) {
    // float error = setpoint - input;
    
    // Avoid division by zero
    if (dt <= 0.0f) return m_pidOutput;
  
    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_pidData.eLast = error;
    }
    
    m_pidData.pError  = error;

    // Calculate integral error with anti-windup
    m_pidData.iError += error * dt;
    m_pidData.iError = std::clamp(m_pidData.iError, (-m_integralMax), m_integralMax);

    // Calculate derivative error and apply exponential moving average filter
    float rawDError = (error - m_pidData.eLast) / dt;
    m_pidData.eLast = error;
    m_pidData.dError = derivativeFilter(rawDError, m_pidData.dError, dt);
    
    // PID output calculation
    // Apply feedforward m_feedForward
    // This is a simple linear feedforward based on the setpoint
	m_pidOutput = (m_pidData.pError * m_kp) + 
                  (m_pidData.iError * m_ki) + 
                  (m_pidData.dError * m_kd) +
                  (m_feedForward * feedForwardVal);

    m_pidOutput = std::clamp(m_pidOutput, (-m_maxOutput), m_maxOutput);
	return m_pidOutput;
}

void SimplePIDController::reset() {
	m_pidData.clear();
    m_pidOutput = 0.0f;
    m_started = false;
}

void SimplePIDController::setDerivativeFilterCoeff(float coeff) {
    m_derivativeTau = std::clamp(coeff, 0.0f, 1.0f);
}