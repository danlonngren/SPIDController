#pragma once

#include "../include/simplePIDControllerInterface.h"

class SimplePIDController : public SimplePIDControllerInterface {
private:
    PidState m_pidData;
    float m_kp, m_ki, m_kd;
    float m_pidOutput;
    float m_maxOutput;

    bool m_started = false;

public:
    SimplePIDController(float kp, float ki, float kd, float outMax);
    ~SimplePIDController() = default;

    float evaluate(float input, float setpoint, float dt);
    void reset();
    void setPidGains(float kp, float ki, float kd);
    void setOutputMax(float newOutputMax);
    void print() const;

    // Getters
    float getPidLastOutput() const { return m_pidOutput; }
    const PidState& getState() const { return m_pidData; }

private:
    float limiter(float value, float min, float max);
};
