#include "simplePIDController.h"

#include <iostream>

using namespace std;

SimplePIDController::SimplePIDController(float kp, float ki, float kd, float outMax) :
    m_pidData(), 
    m_kp(kp), 
    m_ki(ki), 
    m_kd(kd), 
    m_maxOutput(outMax), 
    m_pidOutput(0.0f) {}

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
    m_pidData.iError += error * dt;
    m_pidData.dError  = (error - m_pidData.eLast) / dt;
    m_pidData.eLast   = error;
    
    // PID output calculation
	m_pidOutput = (m_pidData.pError * m_kp) + 
                  (m_pidData.iError * m_ki) + 
                  (m_pidData.dError * m_kd);

    m_pidOutput = limiter(m_pidOutput, (-m_maxOutput), m_maxOutput);
	return m_pidOutput;
}

void SimplePIDController::reset() {
	m_pidData.clear();
    m_pidOutput = 0.0f;
    m_started = false;
}

void SimplePIDController::setPidGains(float kp, float ki, float kd) {
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;
}

void SimplePIDController::setOutputMax(float outMax) {
	m_maxOutput = outMax;
}

float SimplePIDController::limiter(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void SimplePIDController::print() const {
    std::cout << "PID Output        : " << m_pidOutput << "\n"
              << "Error             : " << m_pidData.pError << "\n"
              << "Integral Error    : " << m_pidData.iError << "\n"
              << "Derivative Error  : " << m_pidData.dError << "\n"
              << "Last Error        : " << m_pidData.eLast << "\n" << endl;
}