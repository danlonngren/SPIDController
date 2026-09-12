#include "simplePIDController.h"

#include <gtest/gtest.h>
#include <memory>

namespace {

float mock_time{};

float getMockTime() {
    return mock_time;
}

class SimplePIDControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        mock_time = 0.0f;
        controller_ = std::make_unique<SimplePIDController>(
            kDefaultOutputLimit, PIDGains{kDefaultGain, kDefaultGain, kDefaultGain}, &getMockTime);
    }

    float evaluateAfter(float elapsed_time, float input, float setpoint,
                        float feed_forward = 0.0f) {
        mock_time += elapsed_time;
        return controller_->evaluate(input, setpoint, feed_forward);
    }

    static constexpr float kDefaultGain = 1.0f;
    static constexpr float kDefaultOutputLimit = 1000.0f;
    std::unique_ptr<SimplePIDController> controller_;
};

TEST_F(SimplePIDControllerTest, CalculatesProportionalAndIntegralTerms) {
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 5.0f, 10.0f), 10.0f);

    const PIDTerm& state = controller_->getPIDState();
    EXPECT_FLOAT_EQ(state.p, 5.0f);
    EXPECT_FLOAT_EQ(state.i, 5.0f);
    EXPECT_FLOAT_EQ(state.d, 0.0f);
    EXPECT_FLOAT_EQ(state.output, 10.0f);
    EXPECT_FLOAT_EQ(state.last_input, 5.0f);
    EXPECT_FLOAT_EQ(state.last_error, 5.0f);
    EXPECT_FLOAT_EQ(state.last_dt, 1.0f);
}

TEST_F(SimplePIDControllerTest, ClampsOutputToConfiguredLimit) {
    controller_->setOutputLimit(50.0f);

    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 0.0f, 400.0f), 50.0f);

    controller_->reset();
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 0.0f, -400.0f), -50.0f);
}

TEST_F(SimplePIDControllerTest, ClampsIntegralTermToConfiguredLimit) {
    controller_->setPidGains(PIDGains{0.0f, 1.0f, 0.0f});
    controller_->setItegralLimit(10.0f);

    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f), 10.0f);
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f), 10.0f);
    EXPECT_FLOAT_EQ(controller_->getPIDState().i, 10.0f);
}

TEST_F(SimplePIDControllerTest, AddsFeedForwardToPidOutput) {
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f), 20.0f);

    controller_->reset();
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f, 5.0f), 25.0f);
}

TEST_F(SimplePIDControllerTest, ResetClearsPidTermsAndRetainsConfiguration) {
    evaluateAfter(1.0f, 100.0f, 500.0f);
    controller_->reset();

    const PIDTerm& reset_state = controller_->getPIDState();
    EXPECT_FLOAT_EQ(reset_state.p, 0.0f);
    EXPECT_FLOAT_EQ(reset_state.i, 0.0f);
    EXPECT_FLOAT_EQ(reset_state.d, 0.0f);
    EXPECT_FLOAT_EQ(reset_state.output, 0.0f);

    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 5.0f, 10.0f), 10.0f);
}

TEST_F(SimplePIDControllerTest, ResetIntegralClearsOnlyTheIntegralTerm) {
    evaluateAfter(1.0f, 5.0f, 10.0f);
    controller_->resetIntegral();

    EXPECT_FLOAT_EQ(controller_->getPIDState().p, 5.0f);
    EXPECT_FLOAT_EQ(controller_->getPIDState().i, 0.0f);
    EXPECT_FLOAT_EQ(controller_->getPIDState().output, 10.0f);
}

TEST_F(SimplePIDControllerTest, SettersUpdateTheirMatchingGetters) {
    const PIDGains gains{2.0f, 3.0f, 4.0f};

    controller_->setOutputLimit(50.0f);
    controller_->setItegralLimit(25.0f);
    controller_->setPidGains(gains);
    controller_->setDerivativeMode(DerivativeMode::Error);
    controller_->setDerivativeLpGain(0.5f);

    EXPECT_FLOAT_EQ(controller_->getOutputLimit(), 50.0f);
    EXPECT_FLOAT_EQ(controller_->getIntegralLimit(), 25.0f);
    EXPECT_FLOAT_EQ(controller_->getPidGains().kp, gains.kp);
    EXPECT_FLOAT_EQ(controller_->getPidGains().ki, gains.ki);
    EXPECT_FLOAT_EQ(controller_->getPidGains().kd, gains.kd);
    EXPECT_EQ(controller_->getDerivativeMode(), DerivativeMode::Error);
    EXPECT_FLOAT_EQ(controller_->getDerivativeLpGain(), 0.5f);
}

TEST_F(SimplePIDControllerTest, MeasurementDerivativeAvoidsSetpointKick) {
    controller_->setPidGains(PIDGains{1.0f, 0.0f, 1.0f});

    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f), 10.0f);
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 30.0f), 20.0f);
}

TEST_F(SimplePIDControllerTest, ErrorDerivativeRespondsToSetpointChange) {
    controller_->setPidGains(PIDGains{1.0f, 0.0f, 1.0f});
    controller_->setDerivativeMode(DerivativeMode::Error);

    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 20.0f), 10.0f);
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 10.0f, 25.0f), 20.0f);
}

TEST_F(SimplePIDControllerTest, DerivativeFilterSmoothsMeasurementChanges) {
    controller_->setPidGains(PIDGains{1.0f, 0.0f, 1.0f});
    controller_->setDerivativeLpGain(10.0f);

    evaluateAfter(1.0f, 10.0f, 20.0f);
    evaluateAfter(1.0f, 10.0f, 30.0f);
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 15.0f, 30.0f), 14.545455f);
    EXPECT_FLOAT_EQ(evaluateAfter(1.0f, 20.0f, 30.0f), 9.1322317f);
}

TEST(SimplePIDControllerNullClockTest, UsesMinimumTimeStepWhenClockIsNull) {
    SimplePIDController controller(100.0f, PIDGains{1.0f, 0.0f, 0.0f}, nullptr);

    EXPECT_FLOAT_EQ(controller.evaluate(5.0f, 10.0f), 5.0f);
}

} // namespace
