#pragma once

// Switch between derivative modes
enum class DerivativeMode { Measurement, Error };

struct PIDGains
{
    float kp{0.0f};
    float ki{0.0f};
    float kd{0.0f};
};

struct PIDConfig
{
    PIDGains gains;

    float integralLimit{0.0f};
    float outputLimit{0.0f};

    /**
     * @brief Time constant for derivative low-pass filter (seconds).
     * Larger tau = more smoothing.
     * tau = 0 disables filtering.
     */
    float derivativeFilterTau{0.1f};

    DerivativeMode derivativeMode{DerivativeMode::Measurement};
};

struct PIDTerm {
    float p{0.0f};
    float i{0.0f};
    float d{0.0f};
    float output{0.0f};
};

/**
 * @brief Simple PID Controller implementation.
 */
class SimplePIDController {
public:
    /**
     * @brief Constructor for the SimplePIDController.
     * @param config PID Configuration
     */
    SimplePIDController(const PIDConfig& config);

    /**
     * @brief Evaluate the PID controller with the given input and setpoint.
     * @param measurement Current value to control.
     * @param setpoint Desired value to achieve.
     * @param dt Time step since the last evaluation.
     * @param feedForwardVal Optional feedforward contribution (already scaled to output units).
     *        Typically computed from desired setpoint derivatives (e.g., velocity, acceleration)
     *        or other predictable disturbances. Defaults to 0 (no feedforward).
     * @return PID output value, limited to [-m_outputMax, m_outputMax].
     */
    float evaluate(float measurement, float setpoint, float dt, float feedForwardVal=0.0f);

    /**
     * @brief Reset the PID controller state.
     */
    void reset();
    
    /**
     * @brief Reset integral error.
     */
    void resetIntegral();

    void setConfig(const PIDConfig& config) { m_config = config; }

    // Getters
    PIDConfig getPIDConfig() { return m_config; }
    PIDTerm getPIDState() { return m_term; }

private:
    float derivativeFilter(float current, float previous, float dt);

    PIDConfig m_config;

    PIDTerm m_term;
    float m_lastMeasurement{0.0f};

    bool m_started{false};

};
