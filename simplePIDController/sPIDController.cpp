#include "sPIDController.h"

#include <iostream>

sPIDController::sPIDController(float kp, float ki, float kd, float outMax) :
    m_error(0.0f), 
    m_iError(0.0f), 
    m_dError(0.0f), 
    m_eLast(0.0f), 
    m_kp(kp), 
    m_ki(ki), 
    m_kd(kd), 
    m_maxOutput(outMax), 
    m_pidOutput(0.0f) {}

float sPIDController::evaluate(float input, float setpoint, float dt) {
    // Avoid division by zero
    if (dt <= 0.0f) 
        return m_pidOutput;
  
    m_error = setpoint - input;
    
    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_eLast = m_error;
    }

    m_dError = (m_error - m_eLast) / dt;
    m_eLast = m_error;
    
    // Integral error with anti-windup
    m_iError += m_error * dt;
    if (m_iError > m_maxOutput)
        m_iError = m_maxOutput;
    else if (m_iError < -m_maxOutput)
        m_iError = -m_maxOutput;
    
    // PID output calculation
	m_pidOutput = m_error * m_kp + m_iError * m_ki + m_dError * m_kd;
    
    m_pidOutput = limiter(m_pidOutput, (-m_maxOutput), m_maxOutput);
        
	return m_pidOutput;
}

void sPIDController::reset() {
	m_error = m_dError = m_iError = m_eLast = 0.0f;
    m_pidOutput = 0.0f;
    m_started = false;
}

void sPIDController::setPidGains(float kp, float ki, float kd) {
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;
}

void sPIDController::setOutputMax(float outMax) {
	m_maxOutput = outMax;
}

float sPIDController::limiter(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void sPIDController::print() const {
    std::cout << "PID Output: " << m_pidOutput << "\n"
              << "Error: " << m_error << "\n"
              << "Integral Error: " << m_iError << "\n"
              << "Derivative Error: " << m_dError << "\n"
              << "Last Error: " << m_eLast << "\n";
}