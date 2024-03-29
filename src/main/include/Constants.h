#pragma once

#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/trajectory/TrapezoidProfile.h>

#include <units/acceleration.h>

#include <units/angle.h>

#include <units/angular_velocity.h>

#include <units/angular_acceleration.h>

#include <units/length.h>

#include <units/time.h>

#include <units/velocity.h>

#include <units/voltage.h>

#include <units/current.h>

#include <wpi/numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace GlobalConstants
{
    static const std::string kCANivoreBus = "rio";
    static const std::string kRIOBus = "rio";
}

namespace DriveConstants
{
    constexpr int kModuleCount = 3;

    constexpr units::voltage::volt_t kDriveMaxVoltage =
        units::voltage::volt_t(10);
    constexpr units::velocity::meters_per_second_t kMaxSpeed =
        units::meters_per_second_t(12);
    constexpr units::angular_velocity::radians_per_second_t kMaxAngularSpeed =
        units::radians_per_second_t(2 * wpi::numbers::pi);

    // TODO: Get real values for these constants
    const units::meter_t kTrackWidth = 0.432_m; // Distance between centers of right and left wheels on robot
    const units::meter_t kWheelBase = 0.686_m;  // Distance between centers of front and back wheels on robot

    namespace FrontLeftModule
    {
        constexpr int kTopMotorPort = 20;
        constexpr int kBottomMotorPort = 21;
        constexpr int kEncoderPortA = 0;
        constexpr int kEncoderPortB = 1;
        constexpr int kEncoderPortAbs = 2;
        const std::string name = "frontLeft";
        const frc::Translation2d translation =
            frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    }
    namespace FrontRightModule
    {
        constexpr int kTopMotorPort = 22;
        constexpr int kBottomMotorPort = 23;
        constexpr int kEncoderPortA = 3;
        constexpr int kEncoderPortB = 4;
        constexpr int kEncoderPortAbs = 5;
        const std::string name = "frontRight";
        const frc::Translation2d translation =
            frc::Translation2d(kWheelBase / 2, kTrackWidth / 2);
    }
    namespace RearModule
    {
        constexpr int kTopMotorPort = 24;
        constexpr int kBottomMotorPort = 25;
        constexpr int kEncoderPortA = 6;
        constexpr int kEncoderPortB = 7;
        constexpr int kEncoderPortAbs = 8;
        const std::string name = "rear";
        const frc::Translation2d translation =
            frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    }
    namespace Gyro
    {
        constexpr double kP = 0.01 * 6; // original is 0.007
        constexpr double kMAX = 0.6 * 6;
    }
}

namespace ModuleConstants
{
    constexpr units::ampere_t kMaxCurrent = 40.0_A;

    constexpr double kPModuleTurningController = 3.5 / wpi::numbers::pi;
    constexpr double kPModuleDriveController = 0.025; // 0.025;
    constexpr double kMaxTurnOutput = 0.5;

    // Marswars settings
    // constexpr auto ks = .64705_V;
    // constexpr auto kv = 2.2489 * 1_V * 1_s / 1_m;
    // constexpr auto ka = .26974 * 1_V * 1_s * 1_s / 1_m;

    constexpr units::voltage::volt_t ks = 0.57219_V;
    constexpr auto kv = 2.184 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.067772 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kDriveGearRatio = 6.4;
    constexpr double kTurnGearRatio = 28;

    constexpr units::meter_t kDriveWheelDiameter = 4_in;

    constexpr units::radians_per_second_t kMaxAngularVelocity =
        units::radians_per_second_t(wpi::numbers::pi * 100.0); // radians per second

    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration =
        units::radians_per_second_squared_t(wpi::numbers::pi * 2.0 * 100.0); // radians per second squared
}

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;
}
