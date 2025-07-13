
#include "sPIDController.h"

#include <gtest/gtest.h>

class CircularBufferTest : public ::testing::Test {
protected:
    void SetUp() override {
    }
};

// Test: Buffer Creation
TEST_F(CircularBufferTest, createPIDController) {
    sPIDController pid(0.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(pid.evaluate(0.0f, 0.0f, 0.0f), 0.0f);
}