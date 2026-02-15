
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
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(0.0f, 0.0f, 0.1f), 0.0f);
    EXPECT_FLOAT_EQ(pid.getPidOutput(), 0.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputTest) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPidOutput(), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLastOutputTest) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPidOutput(), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerGetStateTest) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputNegTest) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(-5.0f, -10.0f, 0.1f), -5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLimitTest) {
    SimplePIDController pid(100.0f, 100.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(400.0f, 800.0f, 0.1f), 100.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerReset) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    pid.evaluate(100.0f, 500.0f, 0.1f);
    pid.reset();
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetGains) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);

    pid.setPidGains(2.0f, 2.0f, 2.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 11.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetOutputMax) {
    SimplePIDController pid(100.0f, 100.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    pid.setOutputMax(50.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(400.0f, 800.0f, 0.1f), 50.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerIntegralWindupLimit) {
    SimplePIDController pid(10.0f, 1000.0f);
    pid.setPidGains(0.0f, 1.0f, 0.0f);
    float output = 0.0f;
    
    for (int i = 0; i < 100; ++i) {
        output = pid.evaluate(10.0f, 20.0f, 1.0f);
    }

    EXPECT_FLOAT_EQ(output, 10.0f);
}

TEST_F(SimplePIDControllerTests, PIDFeedForwardEffect) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 1.0f, 1.0f);
    float output = 0.0f;
    output = pid.evaluate(10.0f, 20.0f, 1.0f, 10.0f);
    EXPECT_FLOAT_EQ(output, 20.0f);
    pid.reset();
    pid.setFeedForwardGain(0.5f);
    output = pid.evaluate(10.0f, 20.0f, 1.0f, 10.0f);
    EXPECT_FLOAT_EQ(output, 25.0f);
}

TEST_F(SimplePIDControllerTests, PIDDerivativeFilterEffect) {
    SimplePIDController pid(100.0f, 1000.0f);
    pid.setPidGains(1.0f, 0.0f, 1.0f);
    pid.setDerivativeFilterTau(10.0f);
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