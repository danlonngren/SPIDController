
#include "sPIDController.h"

#include <gtest/gtest.h>

class SimplePIDControllerTests : public ::testing::Test {
protected:
    void SetUp() override {
    }
    void TearDown() override {
    }
};

// Test: Buffer Creation
TEST_F(SimplePIDControllerTests, PIDControllerCreate) {
    sPIDController pid(1.0f, 1.0f, 1.0f, 1000.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(5.0f, 10.0f, 0.1f), 5.5f);
}

TEST_F(SimplePIDControllerTests, PIDControllerLimitTest) {
    sPIDController pid(1.0f, 1.0f, 1.0f, 100.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(100.0f, 500.0f, 0.1f), 100.0f);
}