#include "simplePIDController.h"

#include <algorithm>
#include <iostream>

SimplePIDController::SimplePIDController(float integralMax, float outMax) :
    m_maxOutput(outMax), 
    m_integralMax(integralMax) {}

float SimplePIDController::evaluate(float input, float setpoint, float dt) {
    float error = setpoint - input;
    
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
    m_pidData.dError = exponentialMovingAverage(rawDError, m_pidData.dError, m_derivativeFilterCoeff);
    
    // PID output calculation
	m_pidOutput = (m_pidData.pError * m_kp) + 
                  (m_pidData.iError * m_ki) + 
                  (m_pidData.dError * m_kd);

    // Apply feedforward term
    // This is a simple linear feedforward based on the setpoint
    m_pidOutput += m_feedForward * setpoint;

    m_pidOutput = std::clamp(m_pidOutput, (-m_maxOutput), m_maxOutput);
	return m_pidOutput;
}

void SimplePIDController::reset() {
	m_pidData.clear();
    m_pidOutput = 0.0f;
    m_started = false;
}

void SimplePIDController::setIntegralMax(float integralMax) {
    m_integralMax = integralMax;
}

void SimplePIDController::setPidGains(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void SimplePIDController::setOutputMax(float outMax) {
	m_maxOutput = outMax;
}

void SimplePIDController::setFeedForwardGain(float feedForward) { 
    m_feedForward = feedForward; 
}

void SimplePIDController::setDerivativeFilterCoeff(float coeff) {
    m_derivativeFilterCoeff = std::clamp(coeff, 0.0f, 1.0f);
}

float SimplePIDController::exponentialMovingAverage(float current, float previous, float coeff) {
    return (coeff * current) + ((1.0f - coeff) * previous);
}