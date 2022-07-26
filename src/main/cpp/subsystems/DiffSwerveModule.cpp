#include "subsystems/DiffSwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

// ============================================================================

DiffSwerveModule::DiffSwerveModule(int topMotorChannel, int bottomMotorChannel, int encoderPort, std::string name, std::string CANbus, wpi::log::DataLog &log)
    : m_topMotor(topMotorChannel, CANbus),
      m_bottomMotor(bottomMotorChannel, CANbus),
      m_encoder(encoderPort),
      m_name(name),
      m_log(log)
{
    // Reset motors and encoders
    m_topMotor.ConfigFactoryDefault();
    m_bottomMotor.ConfigFactoryDefault();

    m_topMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage);
    m_topMotor.EnableVoltageCompensation(true);
    m_bottomMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage);
    m_bottomMotor.EnableVoltageCompensation(true);

    SupplyCurrentLimitConfiguration supply{true, ModuleConstants::MAX_CURRENT, ModuleConstants::MAX_CURRENT, 10};
    m_topMotor.ConfigSupplyCurrentLimit(supply);
    m_bottomMotor.ConfigSupplyCurrentLimit(supply);

    // StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
    // m_driveMotor.ConfigStatorCurrentLimit(stator);
    // m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));

    m_topMotor.SetNeutralMode(NeutralMode::Coast);
    m_bottomMotor.SetNeutralMode(NeutralMode::Coast);

    m_topMotorCurrent = wpi::log::DoubleLogEntry(log, "/" + m_name + "/topMotorCurrent");
    m_bottomMotorCurrent = wpi::log::DoubleLogEntry(log, "/" + m_name + "/bottomMotorCurrent");

    m_topMotorRPM = wpi::log::DoubleLogEntry(log, "/" + m_name + "/topMotorRPM");
    m_bottomMotorRPM = wpi::log::DoubleLogEntry(log, "/" + m_name + "/bottomMotorRPM");

    m_wheelSpeed = wpi::log::DoubleLogEntry(log, "/" + m_name + "/wheelSpeed");
    m_moduleAngleLog = wpi::log::DoubleLogEntry(log, "/" + m_name + "/moduleAngle");

    m_expectedSpeed = wpi::log::DoubleLogEntry(log, "/" + m_name + "/expectedSpeed");
    m_expectedAngle = wpi::log::DoubleLogEntry(log, "/" + m_name + "/expectedAngle");
}

// ============================================================================

// called from DriveSubsystem Periodic

frc::SwerveModuleState DiffSwerveModule::GetState()
{

    double topMotorSpeed = m_topMotor.GetSelectedSensorVelocity();
    double bottomMotorSpeed = m_bottomMotor.GetSelectedSensorVelocity();

    m_driveSpeed = GetDriveSpeed(topMotorSpeed, bottomMotorSpeed);
    m_moduleAngle = units::degree_t(m_encoder.GetAbsolutePosition() - 180 - m_offset);

    // data logging
    auto topCurrent = m_topMotor.GetStatorCurrent();
    auto bottomCurrent = m_bottomMotor.GetStatorCurrent();

    m_topMotorCurrent.Append(topCurrent);
    m_bottomMotorCurrent.Append(bottomCurrent);

    m_topMotorRPM.Append(topMotorSpeed);
    m_bottomMotorRPM.Append(bottomMotorSpeed);

    m_wheelSpeed.Append(m_driveSpeed.value());
    m_moduleAngleLog.Append(m_moduleAngle.value());

    return {m_driveSpeed, frc::Rotation2d(m_moduleAngle)};
}

// ============================================================================

units::meters_per_second_t DiffSwerveModule::GetDriveSpeed(double topSpeed, double bottomSpeed)
{
    double speed =
        ((topSpeed - bottomSpeed) / 2.0)   /*Average Sensor Velocity (raw/100ms)*/
        * (10.0 / 2048)                    /*Motor Revolutions per second*/
        * ModuleConstants::kDriveGearRatio /*Output revolutions per second*/
        * ((ModuleConstants::kDriveWheelDiameterInches / 39.37) * wpi::numbers::pi) /*Circumference in meters (meters/second)*/;

    frc::SmartDashboard::PutNumber(m_name + " Wheel Speed ", speed);

    return units::meters_per_second_t(speed);
}

// ============================================================================

double DiffSwerveModule::SetDesiredState(const frc::SwerveModuleState &desiredState)
{

    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
        desiredState, m_moduleAngle);

    m_expectedSpeed.Append(state.speed.value());
    m_expectedAngle.Append(state.angle.Radians().value());

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate(m_driveSpeed.value(), state.speed.value());

    // Calculate the turning motor output from the turning PID controller.
    auto turnOutput{m_turningPIDController.Calculate(m_moduleAngle, state.angle.Radians())};

    // Dont turn at more than 50% power
    turnOutput = std::clamp(turnOutput, -ModuleConstants::kMaxTurnOutput, ModuleConstants::kMaxTurnOutput);

    const auto driveFeedForward{m_driveFeedForward.Calculate(state.speed)};

    m_topVoltage =
        units::volt_t(driveOutput + driveFeedForward.value() + DriveConstants::driveMaxVoltage * turnOutput);

    m_bottomVoltage =
        units::volt_t(-driveOutput - driveFeedForward.value() + DriveConstants::driveMaxVoltage * turnOutput);

    return std::max(m_topVoltage, m_bottomVoltage).value();
}

void DiffSwerveModule::SetVoltage(double driveMax)
{
    m_topMotor.Set(ControlMode::PercentOutput, m_topVoltage.value() * driveMax / DriveConstants::driveMaxVoltage);
    m_bottomMotor.Set(ControlMode::PercentOutput, m_bottomVoltage.value() * driveMax / DriveConstants::driveMaxVoltage);
}

// ============================================================================

void DiffSwerveModule::ResetEncoders()
{
    m_topMotor.SetSelectedSensorPosition(0);
    m_bottomMotor.SetSelectedSensorPosition(0);
}

void DiffSwerveModule::MotorsOff()
{
    m_topMotor.Set(ControlMode::PercentOutput, 0);
    m_bottomMotor.Set(ControlMode::PercentOutput, 0);
}

// =========================Wheel Offsets======================================

void DiffSwerveModule::SetWheelOffset()
{
    auto steerPosition{m_encoder.GetAbsolutePosition() - 180};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
    frc::Preferences::SetDouble(m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================

void DiffSwerveModule::LoadWheelOffset()
{
    auto steerPosition{frc::Preferences::GetDouble(m_name)};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================
