#include <gtest/gtest.h>
#include "../include/algorithms/kinematics.hpp"
#include "../include/nodes/motor.hpp"
#include <cmath>

#include "motor.hpp"

using namespace algorithms;

constexpr float ERROR = 0.001f;
constexpr float WHEEL_BASE = nodes::wheel_base;
constexpr float WHEEL_RADIUS = nodes::wheel_radius;
constexpr float WHEEL_CIRCUMFERENCE = 2 * M_PI * WHEEL_RADIUS;
constexpr int32_t PULSES_PER_ROTATION = nodes::TPR;

TEST(KinematicsTest, BackwardZeroVelocitySI) {
    constexpr float linear = 0.0f;
    constexpr float angular = 0.0f;
    constexpr float expected_l = 0.0f;
    constexpr float expected_r = 0.0f;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveLinearVelocitySI) {
    constexpr float linear = 1.0f;
    constexpr float angular = 0.0f;
    constexpr float expected_l = 1.0f / WHEEL_CIRCUMFERENCE * 2 * M_PI;
    constexpr float expected_r = 1.0f / WHEEL_CIRCUMFERENCE * 2 * M_PI;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, BackwardPositiveAngularVelocitySI) {
    constexpr float linear = 0.0f;
    constexpr float angular = 1.0f;
    constexpr float expected_l = -(0.5f * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);
    constexpr float expected_r = +(0.5f * WHEEL_BASE) / WHEEL_CIRCUMFERENCE * (2 * M_PI);

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.inverse(RobotSpeed{linear, angular});
    EXPECT_NEAR(result.l, expected_l, ERROR);
    EXPECT_NEAR(result.r, expected_r, ERROR);
}

TEST(KinematicsTest, ForwardZeroWheelSpeedSI) {
    constexpr float wheel_l = 0;
    constexpr float wheel_r = 0;
    constexpr float expected_l = 0;
    constexpr float expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardEqualWheelSpeedsSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = WHEEL_RADIUS;
    constexpr float expected_a= 0;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);
}

TEST(KinematicsTest, ForwardOppositeWheelSpeedsSI) {
    constexpr float wheel_l = -1;
    constexpr float wheel_r = 1;
    constexpr float expected_l = 0;
    constexpr float expected_a= (WHEEL_RADIUS / (0.5 * WHEEL_BASE));

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto result = kin.forward(WheelSpeed {wheel_l,wheel_r});
    EXPECT_NEAR(result.v, expected_l, ERROR);
    EXPECT_NEAR(result.w, expected_a, ERROR);;
}

TEST(KinematicsTest, ForwardAndBackwardSI) {
    constexpr float wheel_l = 1;
    constexpr float wheel_r = -0.5;

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto lin_ang = kin.forward(WheelSpeed {wheel_l,wheel_r});
    auto result = kin.inverse(lin_ang);
    EXPECT_NEAR(result.l, wheel_l, ERROR);
    EXPECT_NEAR(result.r, wheel_r, ERROR);
}


TEST(KinematicsTest, ForwardAndBackwardEncoderDiff) {
    constexpr uint32_t encoder_l = 135822;  //aprox. 100m
    constexpr uint32_t encoder_r = 113411;  //aprox. 90m

    Kinematics kin(WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
    auto d_robot_pose = kin.forward(Encoders {encoder_l,encoder_r});
    auto result = kin.inverse(d_robot_pose);
    EXPECT_NEAR(result.l, encoder_l, 1);   //error rate under 20 ticks
    EXPECT_NEAR(result.r, encoder_r, 1);
}

// Main function to run all tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
