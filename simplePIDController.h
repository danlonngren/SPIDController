#pragma once


/**
 * @brief Simple PID Controller implementation.
 */
class SimplePIDController {
public:
    enum class DerivativeMode { Measurement, Error };

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

    // PID gains
    float m_kp;
    float m_ki;
    float m_kd;
    
    float m_maxOutput;
    float m_integralMax;
    
    const DerivativeMode m_derivativeMode;

    // PID State
    PidState m_pidData;
    float m_pidOutput{0.0f};

    bool m_started{false};

    /**
     * @brief Time constant for derivative low-pass filter (seconds).
     * Larger tau = more smoothing.
     * tau = 0 disables filtering.
     */
    float m_derivativeTau{0.1f};

public:
    /**
     * @brief Constructor for the SimplePIDController.  
     */
    SimplePIDController(
        float kp, float ki, float kd, 
        float integralMax, 
        float outMax, 
        DerivativeMode dMode = DerivativeMode::Measurement);
    ~SimplePIDController() = default;

    /**
     * @brief Evaluate the PID controller with the given input and setpoint.
     * @param input Current value to control.
     * @param setpoint Desired value to achieve.
     * @param dt Time step since the last evaluation.
     * @param feedForwardVal Optional feedforward contribution (already scaled to output units).
     *        Typically computed from desired setpoint derivatives (e.g., velocity, acceleration)
     *        or other predictable disturbances. Defaults to 0 (no feedforward).
     * @return PID output value, limited to [-m_maxOutput, m_maxOutput].
     */
    float evaluate(float measurement, float setpoint, float dt, float feedForwardVal=0.0f);

    /**
     * @brief Reset the PID controller state.
     */
    void reset();

    /**
     * @brief Set derivative filter time constant (seconds).
     * @param tau Time constant in seconds. 
     *        tau = 0 -> no filtering
     */
    void setDerivativeFilterTau(float tau);

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
     * @brief Getters
     */
    inline float getIntegralMax() const { return m_integralMax; }
    inline float getOutputMax() const { return m_maxOutput; }
    inline float getPidOutput() const { return m_pidOutput; }

private:
    float derivativeFilter(float current, float previous, float dt);
};
