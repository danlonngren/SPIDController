#pragma once

struct PidState {
    float pError = 0.0f;
    float iError = 0.0f;
    float dError = 0.0f;
    float eLast  = 0.0f;

    void clear() {
        pError = iError = dError = eLast = 0.0f;
    }
};

class IsPIDController {
public:
    virtual ~IsPIDController() = default;
    virtual float evaluate(float input, float setpoint, float dt) = 0;
    virtual void reset() = 0;
    virtual void setPidGains(float kp, float ki, float kd) = 0;
    virtual void setOutputMax(float outMax) = 0;
    virtual float getPidLastOutput() const = 0;
    virtual const struct PidState& getState() const = 0;
};

class SPIDController : public IsPIDController {
private:
    PidState m_pidData;
    float m_kp, m_ki, m_kd;
    float m_pidOutput;
    float m_maxOutput;

    bool m_started = false;

public:
    SPIDController(float kp, float ki, float kd, float outMax);
    ~SPIDController() = default;

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
