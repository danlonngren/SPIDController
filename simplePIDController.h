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

    PidState m_pidData;
    float m_kp, m_ki, m_kd;
    float m_pidOutput;
    float m_maxOutput;
    float m_integralMax;


    bool m_started = false;

public:
    SimplePIDController(float kp, float ki, float kd, float integralMax, float outMax);
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
    void setPidGains(float kp, float ki, float kd);

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
    
private:
    /**
     * @brief Limit the value to a specified range.
     * @param value Value to limit.
     * @param min Minimum limit.
     * @param max Maximum limit.
     * @return Limited value.
     */
    float limiter(float value, float min, float max);
};
