#pragma once

class sPIDController {
private:
    float m_error, m_iError, m_dError;
    float m_eLast;
    float m_kp, m_ki, m_kd;
    float m_pidOutput;
    float m_maxOutput;

    bool m_started = false;

public:
    sPIDController(float kp, float ki, float kd, float outMax);
    ~sPIDController() = default;

    float evaluate(float input, float setpoint, float dt);
    void setPidGains(float kp, float ki, float kd);
    void setOutputMax(float newOutputMax);
    void reset();
    void print() const;

private:
    float limiter(float value, float min, float max);
};
