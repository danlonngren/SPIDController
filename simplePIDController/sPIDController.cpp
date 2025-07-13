#include "sPIDController.h"

#include <iostream>

using namespace std;

sPIDController::sPIDController(float kp, float ki, float kd, float outMax) :
    m_pidData(), 
    m_kp(kp), 
    m_ki(ki), 
    m_kd(kd), 
    m_maxOutput(outMax), 
    m_pidOutput(0.0f) {}

float sPIDController::evaluate(float input, float setpoint, float dt) {
    // Avoid division by zero
    if (dt <= 0.0f) 
        return m_pidOutput;
  
    m_pidData.pError = setpoint - input;
    
    // If we just started, initialize the last error
    // This prevents a large spike in the first output
    if (!m_started) {
        m_started = true;
        m_pidData.eLast = m_pidData.pError;
    }

    m_pidData.dError = (m_pidData.pError - m_pidData.eLast) / dt;
    m_pidData.eLast = m_pidData.pError;
    
    // Integral error with anti-windup
    m_pidData.iError += m_pidData.pError * dt;
    if (m_pidData.iError > m_maxOutput)
        m_pidData.iError = m_maxOutput;
    else if (m_pidData.iError < -m_maxOutput)
        m_pidData.iError = -m_maxOutput;
    
    // PID output calculation
	m_pidOutput = m_pidData.pError * m_kp + m_pidData.iError * m_ki + m_pidData.dError * m_kd;
    m_pidOutput = limiter(m_pidOutput, (-m_maxOutput), m_maxOutput);
        
	return m_pidOutput;
}

void sPIDController::reset() {
	m_pidData.clear();
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
    std::cout << "PID Output        : " << m_pidOutput << "\n"
              << "Error             : " << m_pidData.pError << "\n"
              << "Integral Error    : " << m_pidData.iError << "\n"
              << "Derivative Error  : " << m_pidData.dError << "\n"
              << "Last Error        : " << m_pidData.eLast << "\n" << endl;
}