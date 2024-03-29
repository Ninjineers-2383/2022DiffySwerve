#include "subsystems/DrivetrainSubsystem.h"

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

DrivetrainSubsystem::DrivetrainSubsystem(wpi::log::DataLog &log)
    : m_frontLeft{FrontLeftModule::kTopMotorPort, FrontLeftModule::kBottomMotorPort,
                  FrontLeftModule::kEncoderPortA, FrontLeftModule::kEncoderPortB, FrontLeftModule::kEncoderPortAbs,
                  FrontLeftModule::name, kCANivoreBus, log},
      m_frontRight{FrontRightModule::kTopMotorPort, FrontRightModule::kBottomMotorPort,
                   FrontRightModule::kEncoderPortA, FrontRightModule::kEncoderPortB, FrontRightModule::kEncoderPortAbs,
                   FrontRightModule::name, kCANivoreBus, log},
      m_rear{RearModule::kTopMotorPort, RearModule::kBottomMotorPort,
             RearModule::kEncoderPortA, RearModule::kEncoderPortB, RearModule::kEncoderPortAbs,
             RearModule::name, kCANivoreBus, log},
      m_pigeonSim{m_pigeon.GetSimCollection()},
      m_odometry{kDriveKinematics, GetHeading(), frc::Pose2d()},
      m_lastPose{m_odometry.GetPose()},
      m_log(log),
      m_fieldCentric{false}
{
    LoadWheelOffsets();

    frc::SmartDashboard::PutData("Field", &m_field);

    AddChild(FrontLeftModule::name, &m_frontLeft);
    AddChild(FrontRightModule::name, &m_frontRight);
    AddChild(RearModule::name, &m_rear);

    if (frc::RobotBase::IsSimulation())
    {
        m_odometry.ResetPosition(frc::Pose2d(1_m, 1_m, 0_deg), 0_deg);
    }
}

void DrivetrainSubsystem::Periodic()
{
    frc::SwerveModuleState moduleStates[kModuleCount];
    char moduleIndex = 0;

    for (auto *module : moduleArray)
    {
        moduleStates[moduleIndex++] = module->GetState();
    }

    auto [vx, vy, vr] = kDriveKinematics.ToChassisSpeeds(moduleStates[0], moduleStates[1], moduleStates[2]);
    m_vr = vr;

    m_currentYaw = m_pigeon.GetYaw() - m_zero;

    m_odometry.Update(
        GetHeading(),
        moduleStates[0],
        moduleStates[1],
        moduleStates[2]);

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
        frc::DataLogManager::Log("INFO: User Button Pressed\nSetting all module rotation offsets\n");
    }

    if (m_counter > 0)
        m_counter -= 1;
}

void DrivetrainSubsystem::SimulationPeriodic()
{
    m_frontLeft.Simulate();
    m_rear.Simulate();
    m_frontRight.Simulate();

    m_pigeonSim.AddHeading(units::degrees_per_second_t(m_vr).value() * 0.02);
}

void DrivetrainSubsystem::Drive(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot)
{
    auto states = kDriveKinematics.ToSwerveModuleStates(
        m_fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                             xSpeed, ySpeed, rot, GetHeading())
                       : frc::ChassisSpeeds{
                             xSpeed,
                             ySpeed,
                             rot});

    SetModuleStates(states);
}

void DrivetrainSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, kModuleCount> &desiredStates)
{
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kMaxSpeed);

    units::voltage::volt_t driveMax = units::voltage::volt_t(0);

    for (int i = 0; i < kModuleCount; i++)
    {
        const units::voltage::volt_t max = moduleArray[i]->SetDesiredState(desiredStates[i]);
        if (max > driveMax)
        {
            driveMax = max;
        }
    }

    if (driveMax > kDriveMaxVoltage)
        driveMax = units::volt_t{
            kDriveMaxVoltage.value() / driveMax.value()};
    else
        driveMax = units::voltage::volt_t(1);

    for (DiffSwerveModule *module : moduleArray)
        module->SetVoltage(driveMax);
}

void DrivetrainSubsystem::ResetEncoders()
{
    for (DiffSwerveModule *module : moduleArray)
        module->ResetEncoders();
}

units::degree_t DrivetrainSubsystem::GetHeading() const
{
    return units::degree_t(m_currentYaw); // was negated
}

void DrivetrainSubsystem::ZeroHeading()
{
    m_zero = m_pigeon.GetYaw();
}

void DrivetrainSubsystem::SetOffsetHeading(int heading)
{
    m_zero = m_pigeon.GetYaw() - heading;
}

double DrivetrainSubsystem::GetTurnRate()
{
    return m_pigeon.GetRate();
}

frc::Pose2d DrivetrainSubsystem::GetPose()
{
    return m_odometry.GetPose();
}

void DrivetrainSubsystem::ResetOdometry(frc::Pose2d pose)
{

    m_currentYaw = pose.Rotation().Degrees().value();
    SetOffsetHeading(m_currentYaw);

    m_odometry.ResetPosition(pose, frc::Rotation2d(GetHeading()));
}

void DrivetrainSubsystem::MotorsOff()
{
    for (DiffSwerveModule *module : moduleArray)
        module->MotorsOff();
}

void DrivetrainSubsystem::ToggleFieldCentric()
{
    m_fieldCentric = !m_fieldCentric;
}

void DrivetrainSubsystem::SetFieldCentric(bool fieldCentric)
{
    m_fieldCentric = fieldCentric;
}

void DrivetrainSubsystem::GyroCrab(double x, double y, double desiredAngle)
{
    double currentAngle = GetHeading().value();
    while (currentAngle > 180.)
        currentAngle -= 360.;
    while (currentAngle < -180.)
        currentAngle += 360.;

    double twist = (desiredAngle - currentAngle);
    while (twist > 180.0)
        twist -= 360.0;
    while (twist < -180.0)
        twist += 360.0;

    twist = std::clamp(twist * Gyro::kP, -Gyro::kMAX, Gyro::kMAX);

    Drive(units::meters_per_second_t(x), units::meters_per_second_t(y), units::radians_per_second_t(twist));
}

void DrivetrainSubsystem::SetWheelOffsets()
{
    for (DiffSwerveModule *module : moduleArray)
        module->SetZeroOffset();
    frc::DataLogManager::Log("INFO: SetWheelOffsets Complete\n");
}

void DrivetrainSubsystem::LoadWheelOffsets()
{
    for (DiffSwerveModule *module : moduleArray)
        module->LoadZeroOffset();

    frc::DataLogManager::Log("INFO: LoadWheelOffsets Complete\n");
}
