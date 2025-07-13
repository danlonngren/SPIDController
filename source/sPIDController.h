#pragma once

class sPIDController {
private:
    class PIDData {
    private:
        float m_error, m_dError, m_iError;
        float m_eLast;

    public:
        PIDData(float e = 0.0f) : 
                m_error(e), m_dError(0.0f),
                m_iError(0.0f), m_eLast(0.0f) {}

        void update(float e, float dt, float ieLim) {
            if (dt <= 0.0f) return; // Avoid division by zero
            m_error = e;
            m_dError = (m_error - m_eLast) / dt;
            m_eLast = m_error;
            // Integral error with anti-windup
            m_iError += m_error * dt;
            if (m_iError > ieLim)
                m_iError = ieLim;
            else if (m_iError < -ieLim)
                m_iError = -ieLim;
        }

        float getError(float gain = 1.0f)  const { return m_error * gain; }
        float getDError(float gain = 1.0f) const { return m_dError * gain; }
        float getIError(float gain = 1.0f) const { return m_iError * gain; }
        float getPidOutput(float pGain = 1.0f, float dGain = 1.0f, float iGain = 1.0f) const {
            return m_error * pGain + m_dError * dGain + m_iError * iGain;
        }

        void clear() {
            m_error = m_dError = m_iError = m_eLast = 0.0f;
        }
    };

private:
    PIDData m_pidData;
    float m_kp, m_ki, m_kd;
    float m_pidOutput;
    float m_maxOutput;

public:
    sPIDController(float kp, float kd, float ki, float outMax);
    ~sPIDController() = default;

    float evaluate(float input, float setpoint, float dt);
    void setPidGains(float kp, float kd, float ki);
    void setOutputMax(float newOutputMax);
    void reset();

private:
    float limiter(float value, float min, float max);
};
