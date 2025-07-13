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
    virtual void print() const = 0;
    virtual const struct PidState& getState() const = 0;
};