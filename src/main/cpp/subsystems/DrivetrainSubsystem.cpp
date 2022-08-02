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
#include <frc/RobotBase.h>

using namespace DriveConstants;
using namespace GlobalConstants;

// ==========================================================================

DrivetrainSubsystem::DrivetrainSubsystem(wpi::log::DataLog &log)
    : m_frontLeft{kFrontLeftTopMotorPort, kFrontLeftBottomMotorPort, kFrontLeftEncoderPot, "frontLeft", kCANivoreBus, log},
      m_rear{kRearTopMotorPort, kRearBottomMotorPort, kRearEncoderPot, "rear", kCANivoreBus, log},
      m_frontRight{kFrontRightTopMotorPort, kFrontRightBottomMotorPort, kFrontRightEncoderPot, "frontRight", kCANivoreBus, log},
      m_pigeonSim{m_pigeon.GetSimCollection()},
      m_odometry{kDriveKinematics, GetHeading(), frc::Pose2d()},
      m_lastPose{m_odometry.GetPose()},
      m_log(log),
      m_fieldCentric{false}
{
    LoadWheelOffsets();

    frc::SmartDashboard::PutData("Field", &m_field);

    if (frc::RobotBase::IsSimulation())
    {
        m_odometry.ResetPosition(frc::Pose2d(1_m, 1_m, 0_deg), 0_deg);
    }
}

// ==========================================================================

void DrivetrainSubsystem::Periodic()
{
    auto frontLeftState = m_frontLeft.GetState();
    auto rearState = m_rear.GetState();
    auto frontRightState = m_frontRight.GetState();

    auto [vx, vy, vr] = kDriveKinematics.ToChassisSpeeds(frontLeftState, frontRightState, rearState);
    m_vr = vr;

    m_currentYaw = m_pigeon.GetYaw() - m_zero;

    m_odometry.Update(
        GetHeading(),
        frontLeftState,
        frontRightState,
        rearState);

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
    m_rear.Simulate();
    m_frontRight.Simulate();

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

void DrivetrainSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 3> &desiredStates)
{
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);

    double flMax = m_frontLeft.SetDesiredState(desiredStates[0]);
    double frMax = m_frontRight.SetDesiredState(desiredStates[1]);
    double bMax = m_rear.SetDesiredState(desiredStates[2]);

    double driveMax = std::max(bMax, std::max(flMax, frMax));

    if (driveMax > DriveConstants::driveMaxVoltage)
        driveMax = DriveConstants::driveMaxVoltage / driveMax;
    else
        driveMax = 1;

    m_frontLeft.SetVoltage(driveMax);
    m_frontRight.SetVoltage(driveMax);
    m_rear.SetVoltage(driveMax);
}

// ==========================================================================

void DrivetrainSubsystem::ResetEncoders()
{
    m_frontLeft.ResetEncoders();
    m_rear.ResetEncoders();
    m_frontRight.ResetEncoders();
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
    m_rear.MotorsOff();
    m_frontRight.MotorsOff();
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
    m_frontRight.SetWheelOffset();
    fmt::print("INFO: SetWheelOffsets Complete\n");
}

// ================================================================

void DrivetrainSubsystem::LoadWheelOffsets()
{
    m_frontLeft.LoadWheelOffset();
    m_rear.LoadWheelOffset();
    m_frontRight.LoadWheelOffset();
    fmt::print("INFO: LoadWheelOffsets Complete\n");
}