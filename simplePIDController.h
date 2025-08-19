#pragma once

/**
 * @brief PID gains structure.
 */
struct PidGains {
    float kp;
    float ki;
    float kd;

    PidGains(float p = 0.0f, float i = 0.0f, float d = 0.0f) : kp(p), ki(i), kd(d) {}
};


/**
 * @brief Simple PID Controller implementation.
 */
class SimplePIDController {
private:
    struct PidState {
        float pError = 0.0f;
        float iError = 0.0f;
        float dError = 0.0f;
        float eLast  = 0.0f;

        void clear() {
            pError = iError = dError = eLast = 0.0f;
        }
    };

    PidState m_pidData;
    PidGains m_pidGains;
    float m_pidOutput;
    float m_maxOutput;
    float m_integralMax;

    bool m_started = false;

    /**
     * @brief Feedforward term to be added to the PID output.
     * This is a simple linear feedforward based on the setpoint.
     * Range: 0-1, where 0 means no feedforward and 1 means full feedforward.
     */
    float m_feedForward = 0.0f;

    /**
     * @brief Coefficient for the derivative filter.
     * This is used to smooth the derivative term using exponetial moving average.
     */
    float m_derivativeFilterCoeff = 0.0f;

public:
    /**
     * @brief Constructor for the SimplePIDController.  
     */
    SimplePIDController(PidGains pidGains, float integralMax, float outMax);
    ~SimplePIDController() = default;

    /**
     * @brief Evaluate the PID controller with the given input and setpoint.
     * @param input Current value to control.
     * @param setpoint Desired value to achieve.
     * @param dt Time step since the last evaluation.
     * @return PID output value, limited to the maximum output.
     */
    float evaluate(float input, float setpoint, float dt);

    /**
     * @brief Reset the PID controller state.
     */
    void reset();

    /**
     * @brief Set the PID gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setPidGains(PidGains pidGains);

    /**
     * @brief Set the maximum output limit for the PID controller.
     * @param integralMax Maximum integral error value.
     */
    void setIntegralMax(float integralMax);

    /**
     * @brief Set the maximum output limit for the PID controller.
     * @param outMax Maximum output value.
     */
    void setOutputMax(float newOutputMax);

    /**
     * @brief Set the feedforward term.
     * @param feedForward Feedforward value to add to the PID output (Range 0-1). 
     * @note 0 means no feedforward, 1 means full feedforward.
     */
    void setFeedForwardGain(float feedForward);

    /**
     * @brief Set the coefficient for the derivative filter.
     * @param coeff Coefficient for the derivative filter (0-1).
     * @note  0 means full filtering, 1 means no filtering.
     */
    void setDerivativeFilterCoeff(float coeff);

    /**
     * @brief Get the maximum integral error value.
     * @return Maximum integral error value.
     */
    inline float getIntegralMax() const { return m_integralMax; }

    /**
     * @brief Get the maximum output limit for the PID controller.
     * @return Maximum output value.
     */
    inline float getOutputMax() const { return m_maxOutput; }

    /**
     * @brief Get the last PID output value.
     */
    inline float getPidOutput() const { return m_pidOutput; }
    
    /**
     * @brief Get the feedforward term.
     * @return Feedforward value.
     */
    inline float getFeedForward() const { return m_feedForward; }
private:
    /**
     * @brief Limit the value to a specified range.
     * @param value Value to limit.
     * @param min Minimum limit.
     * @param max Maximum limit.
     * @return Limited value.
     */
    float limiter(float value, float min, float max);

    float exponentialMovingAverage(float current, float previous, float coeff);
};
