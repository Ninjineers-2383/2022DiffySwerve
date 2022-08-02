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
    constexpr int kFrontLeftTopMotorPort = 20;
    constexpr int kFrontLeftBottomMotorPort = 21;
    constexpr int kFrontLeftEncoderPot = 1;

    constexpr int kFrontRightTopMotorPort = 22;
    constexpr int kFrontRightBottomMotorPort = 23;
    constexpr int kFrontRightEncoderPot = 2;

    constexpr int kRearTopMotorPort = 24;
    constexpr int kRearBottomMotorPort = 25;
    constexpr int kRearEncoderPot = 3;

    constexpr double driveMaxVoltage = 10;

    constexpr auto kMaxSpeed = units::meters_per_second_t(4);
} // namespace DriveConstants

// ==========================================================================

namespace ModuleConstants
{
    constexpr double MAX_CURRENT = 80.0;

    constexpr double kPModuleTurningController = 1 / wpi::numbers::pi;
    // constexpr double kPModuleTurningController = 100;
    constexpr double kPModuleDriveController = 0.5; // 0.025;
    constexpr double kMaxTurnOutput = .5;

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
} // namespace ModuleConstants

// ==========================================================================

namespace OIConstants
{
    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;
}

// ==========================================================================