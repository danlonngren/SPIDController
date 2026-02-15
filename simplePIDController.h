#pragma once


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

    float m_maxOutput;
    float m_integralMax;
    
    // PID State
    PidState m_pidData;
    float m_pidOutput{0.0f};

    // PID gains
    float m_kp{1.0f};
    float m_ki{1.0f};
    float m_kd{1.0f};


    bool m_started{false};

    /**
     * @brief Feedforward term to be added to the PID output.
     * This is a simple linear feedforward based on the setpoint.
     * Range: 0-1, where 0 means no feedforward and 1 means full feedforward.
     */
    float m_feedForward{0.0f};

    /**
     * @brief Coefficient for the derivative filter.
     * This is used to smooth the derivative term using exponential moving average.
     */
    float m_derivativeFilterCoeff{1.0f};

public:
    /**
     * @brief Constructor for the SimplePIDController.  
     */
    SimplePIDController(float integralMax, float outMax);
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
     * @brief Set the coefficient for the derivative filter.
     * @param coeff Coefficient for the derivative filter (0-1).
     * @note  0 means full filtering, 1 means no filtering.
     */
    void setDerivativeFilterCoeff(float coeff);

    /**
     * @brief Set the PID gains.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     */
    void setPidGains(float kp, float ki, float kd) {
        m_kp = kp; m_ki = ki; m_kd = kd;
    }

    /**
     * @brief Set the maximum output limit for the PID controller.
     * @param integralMax Maximum integral error value.
     */
    void setIntegralMax(float integralMax) { m_integralMax = integralMax; }

    /**
     * @brief Set the maximum output limit for the PID controller.
     * @param outMax Maximum output value.
     */
    void setOutputMax(float newOutputMax) { m_maxOutput = newOutputMax; }

    /**
     * @brief Set the feedforward term.
     * @param feedForward Feedforward value to add to the PID output (Range 0-1). 
     * @note 0 means no feedforward, 1 means full feedforward.
     */
    void setFeedForwardGain(float feedForward) { m_feedForward = feedForward; }

    /**
     * @brief Getters
     */
    inline float getIntegralMax() const { return m_integralMax; }
    inline float getOutputMax() const { return m_maxOutput; }
    inline float getPidOutput() const { return m_pidOutput; }
    inline float getFeedForward() const { return m_feedForward; }

private:

    float exponentialMovingAverage(float current, float previous, float coeff) {
        return (coeff * current) + ((1.0f - coeff) * previous);
    }
};
