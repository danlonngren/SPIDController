#pragma once

// Switch between derivative modes
enum class DerivativeMode { Measurement, Error };

struct PIDGains {
    float kp{0.0f};
    float ki{0.0f};
    float kd{0.0f};
};

struct PIDTerm {
    float p{};
    float i{};
    float d{};
    float output{};
    float last_input{};
    float last_error{};
    float last_dt{};
};

// Function pointer for get time to base calculations from.
using GetTime = float (*)();

/**
 * @brief Simple PID Controller implementation.
 */
class SimplePIDController {
public:
    /**
     * @brief Constructor for the SimplePIDController.
     * @param config PID Configuration
     * @param get_time Funtion pointer to platforms get time function (Returns type float)
     */
    explicit SimplePIDController(float output_limit, PIDGains gains, GetTime get_time);

    /**
     * @brief Evaluate the PID controller with the given input and setpoint.
     * @param input Current value to control.
     * @param setpoint Desired value to achieve.
     * @param feed_forward_val Optional feedforward contribution (already scaled to output units).
     *        Typically computed from desired setpoint derivatives (e.g., velocity, acceleration)
     *        or other predictable disturbances. Defaults to 0 (no feedforward).
     * @return PID output value, limited to [-m_outputMax, m_outputMax].
     */
    float evaluate(float input, float setpoint, float feed_forward_val = 0.0f);

    /**
     * @brief Reset the PID controller state.
     */
    void reset();

    /**
     * @brief Reset integral error.
     */
    void resetIntegral();

    // Setters
    void setOutputLimit(float output_limit) {
        m_config.output_limit = output_limit;
    }

    void setItegralLimit(float integral_limit) {
        m_config.integral_limit = integral_limit;
    }

    void setPidGains(const PIDGains& gains) {
        m_gains = gains;
    }

    void setDerivativeMode(DerivativeMode mode) {
        m_config.derivative_mode = mode;
    }

    void setDerivativeLpGain(float tau) {
        m_config.derivative_filter_tau = tau;
    }

    // Getters
    float getOutputLimit() const {
        return m_config.output_limit;
    }

    float getIntegralLimit() const {
        return m_config.integral_limit;
    }

    const PIDGains& getPidGains() const {
        return m_gains;
    }

    const PIDTerm& getPIDState() const {
        return m_term;
    }

    DerivativeMode getDerivativeMode() const {
        return m_config.derivative_mode;
    }

    float getDerivativeLpGain() const {
        return m_config.derivative_filter_tau;
    }

private:
    // Private memebers
    float derivativeFilter(float current, float previous, float dt) const;

    float getTimeSafe();

    // Private structs
    struct PIDConfig {
        float integral_limit{0.0f};
        float output_limit{0.0f};

        /**
         * @brief Time constant for derivative low-pass filter (seconds).
         * Larger > more smoothing. 0 Disabled.
         */
        float derivative_filter_tau{0.0f};

        DerivativeMode derivative_mode{DerivativeMode::Measurement};
    };

    // Private variables
    PIDGains m_gains;
    GetTime m_time_func;

    PIDConfig m_config;
    PIDTerm m_term;

    bool m_started{false};

    static constexpr float m_min_dt = 1e-6f;
};
