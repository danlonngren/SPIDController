#pragma once

class sPIDController {
private:
    struct pidData {
        pidData() : pError(0.0f), iError(0.0f), dError(0.0f), eLast(0.0f) {}
        ~pidData() = default;
        
        float pError, iError, dError, eLast;

        void clear() {
            pError = iError = dError = eLast = 0.0f;
        }
    };

    pidData m_pidData;
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
