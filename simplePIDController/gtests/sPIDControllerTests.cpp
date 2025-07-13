
#include "sPIDController.h"

#include <gtest/gtest.h>

class SimplePIDControllerTests : public ::testing::Test {
protected:
    void SetUp() override {
    }
    void TearDown() override {
    }
};

TEST_F(SimplePIDControllerTests, PIDControllerOutputZeroTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 5.0f, 0.1f), 0.0f);
    EXPECT_FLOAT_EQ(pid.getPidLastOutput(), 0.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPidLastOutput(), 5.5f);
    EXPECT_FLOAT_EQ(pid.getState().pError, 5.0f);
    EXPECT_FLOAT_EQ(pid.getState().iError, 0.5f);
    EXPECT_FLOAT_EQ(pid.getState().dError, 0.0f);
    EXPECT_FLOAT_EQ(pid.getState().eLast, 5.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLastOutputTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getPidLastOutput(), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerGetStateTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
    EXPECT_FLOAT_EQ(pid.getState().pError, 5.0f);
    EXPECT_FLOAT_EQ(pid.getState().iError, 0.5f);
    EXPECT_FLOAT_EQ(pid.getState().dError, 0.0f);
    EXPECT_FLOAT_EQ(pid.getState().eLast, 5.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerOutputNegTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(10.0f, 5.0f, 0.1f), -5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLimitTest) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 100.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(100.0f, 500.0f, 0.1f), 100.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerReset) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    pid.evaluate(100.0f, 500.0f, 0.1f);
    pid.reset();
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetGains) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    pid.setPidGains(2.0f, 2.0f, 2.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 11.0f);
}

TEST_F(SimplePIDControllerTests, PIDControllerSetOutputMax) {
    SPIDController pid(1.0f, 1.0f, 1.0f, 100.0f);
    pid.setOutputMax(50.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(100.0f, 500.0f, 0.1f), 50.0f);
}