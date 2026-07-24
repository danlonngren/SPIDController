
#include "simplePIDController.h"

#include <gtest/gtest.h>

class SimplePIDControllerTests : public ::testing::Test {
protected:
    void SetUp() override {
    }
    void TearDown() override {
    }
};

TEST_F(SimplePIDControllerTests, PIDControllerOutputZeroTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(0.0f, 0.0f, 0.1f), 0.0f);
    EXPECT_FLOAT_EQ(pid.getPIDState().output, 0.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPIDState().output, 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLastOutputTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPIDState().output, 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerGetStateTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputNegTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(-5.0f, -10.0f, 0.1f), -5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLimitTest) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 100.0f };
    SimplePIDController pid(config);
    EXPECT_FLOAT_EQ(pid.evaluate(400.0f, 800.0f, 0.1f), 100.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerReset) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    pid.evaluate(100.0f, 500.0f, 0.1f);
    pid.reset();
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetGains) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);

    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    config.gains.kp = 2.0f;
    config.gains.ki = 2.0f;
    config.gains.kd = 2.0f;
    pid.setConfig(config);
    pid.reset();
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 11.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetOutputMax) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 100.0f };
    SimplePIDController pid(config);
    config.outputLimit = 50.0f;
    pid.setConfig(config);
    EXPECT_FLOAT_EQ(pid.evaluate(400.0f, 800.0f, 0.1f), 50.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerIntegralWindupLimit) {
    PIDConfig config{0.0f, 1.0f, 1.0f, 10.0f, 100.0f };
    SimplePIDController pid(config);
    float output = 0.0f;
    
    for (int i = 0; i < 100; ++i) {
        output = pid.evaluate(10.0f, 20.0f, 1.0f);
    }

    EXPECT_FLOAT_EQ(output, 10.0f);
}

TEST_F(SimplePIDControllerTests, PIDFeedForwardEffect) {
    PIDConfig config{1.0f, 1.0f, 1.0f, 100.0f, 1000.0f };
    SimplePIDController pid(config);
    float output = 0.0f;
    output = pid.evaluate(10.0f, 20.0f, 1.0f, 0.0f);
    EXPECT_FLOAT_EQ(output, 20.0f);
    pid.reset();
    output = pid.evaluate(10.0f, 20.0f, 1.0f, 0.5f * 10.0f);
    EXPECT_FLOAT_EQ(output, 25.0f);
}

TEST_F(SimplePIDControllerTests, PIDDerivativeFilterEffect) {
    PIDConfig config{1.0f, 0.0f, 1.0f, 100.0f, 1000.0f, 10.0f };
    SimplePIDController pid(config);

    float output = 0.0f;
    output = pid.evaluate(10.0f, 20.0f, 1.0f);
    EXPECT_FLOAT_EQ(output, 10.0f);

    output = pid.evaluate(10.0f, 30.0f, 1.0f);
    EXPECT_FLOAT_EQ(output, 20.0f);

    output = pid.evaluate(15.0f, 30.0f, 1.0f);
    EXPECT_FLOAT_EQ(output, 14.545455f);

    output = pid.evaluate(20.0f, 30.0f, 1.0f);
    EXPECT_FLOAT_EQ(output, 9.1322317f);
}