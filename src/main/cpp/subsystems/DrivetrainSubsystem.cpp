// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DrivetrainSubsystem.h"
//#include <frc/geometry/Rotation2d.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "Constants.h"
#include <wpi/numbers>
#include <math.h>

using namespace DriveConstants;
using namespace GlobalConstants;

// ==========================================================================

DrivetrainSubsystem::DrivetrainSubsystem(wpi::log::DataLog &log)
    : m_frontLeft{kFrontLeftTopMotorPort, kFrontLeftBottomMotorPort, kFrontLeftEncoderPot, "frontLeft", kCANivoreBus, log},
      m_rearLeft{kRearLeftTopMotorPort, kRearLeftBottomMotorPort, kRearLeftEncoderPot, "rearLeft", kCANivoreBus, log},
      m_frontRight{kFrontRightTopMotorPort, kFrontRightBottomMotorPort, kFrontRightEncoderPot, "frontRight", kCANivoreBus, log},
      m_rearRight{kRearRightTopMotorPort, kRearRightBottomMotorPort, kRearRightEncoderPot, "rearRight", kCANivoreBus, log},
      m_pigeonSim{m_pigeon.GetSimCollection()},
      m_odometry{kDriveKinematics, GetHeading(), frc::Pose2d()},
      m_lastPose{m_odometry.GetPose()},
      m_log(log),
      m_fieldCentric{false}
{
    LoadWheelOffsets();

    frc::SmartDashboard::PutData("Field", &m_field);
}

// ==========================================================================

void DrivetrainSubsystem::Periodic()
{
    auto frontLeftState = m_frontLeft.GetState();
    auto rearLeftState = m_rearLeft.GetState();
    auto frontRightState = m_frontRight.GetState();
    auto rearRightState = m_rearRight.GetState();

    auto [vx, vy, vr] = kDriveKinematics.ToChassisSpeeds(frontLeftState, frontRightState, rearLeftState, rearRightState);
    m_vr = vr;

    m_currentYaw = m_pigeon.GetYaw() - m_zero;

    m_odometry.Update(
        GetHeading(),
        frontLeftState,
        frontRightState,
        rearLeftState,
        rearRightState);

    frc::SmartDashboard::PutNumber("Gyro", m_currentYaw);
    frc::SmartDashboard::PutBoolean("FieldCentric", m_fieldCentric);

    frc::SmartDashboard::PutNumber("vx", vx.value());
    frc::SmartDashboard::PutNumber("vy", vy.value());
    frc::SmartDashboard::PutNumber("vr", vr.value());

    m_field.SetRobotPose(m_odometry.GetPose());

    // Press the USER button the RIO to reset the offset on all swerve modules
    if (frc::RobotController::GetUserButton() == 1 && m_counter == 0)
    {
        SetWheelOffsets();
        m_counter = 100;
        fmt::print("INFO: User Button Pressed\nSetting all module rotation offsets\n");
    }

    if (m_counter > 0)
    {
        m_counter -= 1;
    }
}

void DrivetrainSubsystem::SimulationPeriodic()
{
    m_frontLeft.Simulate();
    m_rearLeft.Simulate();
    m_frontRight.Simulate();
    m_rearRight.Simulate();

    m_pigeonSim.AddHeading(units::degrees_per_second_t{m_vr}.value() * 0.02);
}

// ==========================================================================

void DrivetrainSubsystem::Drive(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot)
{
    auto states = kDriveKinematics.ToSwerveModuleStates(
        m_fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                             xSpeed, ySpeed, rot, GetHeading())
                       : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

    SetModuleStates(states);
}

// ==========================================================================

void DrivetrainSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> &desiredStates)
{
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);

    double flMax = m_frontLeft.SetDesiredState(desiredStates[0]);
    double frMax = m_frontRight.SetDesiredState(desiredStates[1]);
    double blMax = m_rearLeft.SetDesiredState(desiredStates[2]);
    double brMax = m_rearRight.SetDesiredState(desiredStates[3]);

    double driveMax = std::max(std::max(blMax, brMax), std::max(flMax, frMax));

    if (driveMax > DriveConstants::driveMaxVoltage)
        driveMax = DriveConstants::driveMaxVoltage / driveMax;
    else
        driveMax = 1;

    m_frontLeft.SetVoltage(driveMax);
    m_frontRight.SetVoltage(driveMax);
    m_rearLeft.SetVoltage(driveMax);
    m_rearRight.SetVoltage(driveMax);
}

// ==========================================================================

void DrivetrainSubsystem::ResetEncoders()
{
    m_frontLeft.ResetEncoders();
    m_rearLeft.ResetEncoders();
    m_frontRight.ResetEncoders();
    m_rearRight.ResetEncoders();
}

// ==========================================================================

units::degree_t DrivetrainSubsystem::GetHeading() const
{
    return units::degree_t(m_currentYaw); // was negated
}

// ==========================================================================

void DrivetrainSubsystem::ZeroHeading()
{
    m_zero = m_pigeon.GetYaw();
}

// ==========================================================================

void DrivetrainSubsystem::SetOffsetHeading(int heading)
{
    m_zero = m_pigeon.GetYaw() - heading;
}

// ==========================================================================

double DrivetrainSubsystem::GetTurnRate()
{
    return m_pigeon.GetRate();
}

// ==========================================================================

frc::Pose2d DrivetrainSubsystem::GetPose()
{
    return m_odometry.GetPose();
}

// ==========================================================================

void DrivetrainSubsystem::ResetOdometry(frc::Pose2d pose)
{

    m_currentYaw = pose.Rotation().Degrees().value();
    SetOffsetHeading(m_currentYaw);

    m_odometry.ResetPosition(pose, frc::Rotation2d(GetHeading()));
}

// ==========================================================================

void DrivetrainSubsystem::MotorsOff()
{
    m_frontLeft.MotorsOff();
    m_rearLeft.MotorsOff();
    m_frontRight.MotorsOff();
    m_rearRight.MotorsOff();
}

// ==========================================================================

void DrivetrainSubsystem::ToggleFieldCentric()
{
    m_fieldCentric = !m_fieldCentric;
}

void DrivetrainSubsystem::SetFieldCentric(bool fieldCentric)
{
    m_fieldCentric = fieldCentric;
}

// ==========================================================================

void DrivetrainSubsystem::GyroCrab(double x, double y, double desiredAngle)
{
    double currentAngle = GetHeading().value();
    while (currentAngle > 180.)
        currentAngle -= 360.;
    while (currentAngle < -180.)
        currentAngle += 360.;

    auto twist = (desiredAngle - currentAngle);

    while (twist > 180.0)
    {
        twist -= 360.0;
    }
    while (twist < -180.0)
    {
        twist += 360.0;
    }

    constexpr double GYRO_P = 0.01 * 6; // original is 0.007
    constexpr double GYRO_MAX = 0.6 * 6;

    twist = std::clamp(twist * GYRO_P, -GYRO_MAX, GYRO_MAX);

    Drive(units::meters_per_second_t(x), units::meters_per_second_t(y), units::radians_per_second_t(twist));
}

// ================================================================

void DrivetrainSubsystem::SetWheelOffsets()
{
    m_frontLeft.SetWheelOffset();
    m_rearLeft.SetWheelOffset();
    m_frontRight.SetWheelOffset();
    m_rearRight.SetWheelOffset();
    fmt::print("INFO: SetWheelOffsets Complete\n");
}

// ================================================================

void DrivetrainSubsystem::LoadWheelOffsets()
{
    m_frontLeft.LoadWheelOffset();
    m_rearLeft.LoadWheelOffset();
    m_frontRight.LoadWheelOffset();
    m_rearRight.LoadWheelOffset();
    fmt::print("INFO: LoadWheelOffsets Complete\n");
}