#include "sPIDController.h"

sPIDController::sPIDController(float kp, float kd, float ki, float outMax) :
    m_pidData(0.0f), m_kp(kp), m_ki(ki), 
    m_kd(kd), m_maxOutput(outMax), m_pidOutput(0.0f) {}

float sPIDController::evaluate(float input, float setpoint, float dt) {
    m_pidData.update(setpoint - input, dt, m_maxOutput);

	m_pidOutput = m_pidData.getPidOutput(m_kp, m_kd, m_ki);

    m_pidOutput = limiter(m_pidOutput, -m_maxOutput, m_maxOutput);

	return m_pidOutput;
}

void sPIDController::reset() {
	m_pidData.clear();
    m_pidOutput = 0.0f;
}

void sPIDController::setPidGains(float kp, float kd, float ki) {
	m_kp = kp;
	m_kd = kd;
	m_ki = ki;
}

void sPIDController::setOutputMax(float outMax) {
	m_maxOutput = outMax;
}

float sPIDController::limiter(float value, float max, float min) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}