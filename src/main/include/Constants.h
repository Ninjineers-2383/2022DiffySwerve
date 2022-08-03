#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
// ===========================================================================

namespace GlobalConstants
{
    static const std::string kCANivoreBus = "Canivore";
    static const std::string kRIOBus = "rio";
}

namespace DriveConstants
{
    constexpr double driveMaxVoltage = 10;
    constexpr auto kMaxSpeed = units::meters_per_second_t(4);

    // TODO: Get real values for these constants
    const units::meter_t kTrackWidth = 0.432_m; // Distance between centers of right and left wheels on robot
    const units::meter_t kWheelBase = 0.686_m;  // Distance between centers of front and back wheels on robot

    namespace FrontLeftModule {
        constexpr int kTopMotorPort = 20;
        constexpr int kBottomMotorPort = 21;
        constexpr int kEncoderPort = 1;
        const std::string name = "frontLeft";
        const frc::Translation2d translation = frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2);
    }
    namespace FrontRightModule {
        constexpr int kTopMotorPort = 22;
        constexpr int kBottomMotorPort = 23;
        constexpr int kEncoderPort = 2;
        const std::string name = "frontRight";
        const frc::Translation2d translation = frc::Translation2d(kWheelBase / 2, kTrackWidth / 2);
    }
    namespace RearLeftModule {
        constexpr int kTopMotorPort = 24;
        constexpr int kBottomMotorPort = 25;
        constexpr int kEncoderPort = 3;
        const std::string name = "rearLeft";
        const frc::Translation2d translation = frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
    }
    namespace RearRightModule {
        constexpr int kTopMotorPort = 24;
        constexpr int kBottomMotorPort = 25;
        constexpr int kEncoderPort = 3;
        const std::string name = "rearRight";
        const frc::Translation2d translation = frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2);
    }
    
    namespace Gyro {
        constexpr double kP = 0.01 * 6; // original is 0.007
        constexpr double kMAX = 0.6 * 6;
    }
} // namespace DriveConstants

// ==========================================================================

namespace ModuleConstants
{
    constexpr double MAX_CURRENT = 80.0;

    constexpr double kPModuleTurningController = 1 / wpi::numbers::pi;
    // constexpr double kPModuleTurningController = 100;
    constexpr double kPModuleDriveController = 0.5; // 0.025;
    constexpr double kMaxTurnOutput = 0.5;

    // DriveFFConstants
    // constexpr auto ks = .64705_V;
    // constexpr auto kv = 2.2489 * 1_V * 1_s / 1_m;
    // constexpr auto ka = .26974 * 1_V * 1_s * 1_s / 1_m;
    
    constexpr auto ks = 0.2_V;
    constexpr auto kv = 0.012 * 1_V * 1_s / 1_m;
    constexpr auto ka = .1 * 1_V * 1_s * 1_s / 1_m;

    constexpr double kDriveGearRatio = 6.4;
    constexpr double kTurnGearRatio = 28;

    constexpr double kDriveWheelDiameterInches = 1.1;

    constexpr units::radians_per_second_t kMaxAngularVelocity =
        units::radians_per_second_t(wpi::numbers::pi * 100.0); // radians per second

    constexpr units::radians_per_second_squared_t kMaxAngularAcceleration =
        units::radians_per_second_squared_t(wpi::numbers::pi * 2.0 * 100.0); // radians per second squared
} // namespace ModuleConstants

// ==========================================================================

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;
}

// ==========================================================================