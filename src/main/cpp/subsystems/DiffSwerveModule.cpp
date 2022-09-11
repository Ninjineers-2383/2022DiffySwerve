#include "subsystems/DiffSwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/voltage.h>

DiffSwerveModule::DiffSwerveModule(int topMotorChannel, int bottomMotorChannel, int encoderPort, std::string name, std::string CANbus, wpi::log::DataLog &log)
    : m_topMotor(topMotorChannel, CANbus),
      m_bottomMotor(bottomMotorChannel, CANbus),
      m_topMotorSim{m_topMotor.GetSimCollection()},
      m_bottomMotorSim{m_bottomMotor.GetSimCollection()},
      m_topMotorSimulator{frc::DCMotor::Falcon500(), 1, units::kilogram_square_meter_t{0.0005}},
      m_bottomMotorSimulator{frc::DCMotor::Falcon500(), 1, units::kilogram_square_meter_t{0.0005}},
      m_encoder(encoderPort),
      m_encoderSim{m_encoder},
      m_name(name),
      m_log(log)
{
    // Reset motors and encoders
    m_topMotor.ConfigFactoryDefault();
    m_bottomMotor.ConfigFactoryDefault();

    m_topMotor.ConfigVoltageCompSaturation(DriveConstants::kDriveMaxVoltage.value());
    m_topMotor.EnableVoltageCompensation(true);
    m_bottomMotor.ConfigVoltageCompSaturation(DriveConstants::kDriveMaxVoltage.value());
    m_bottomMotor.EnableVoltageCompensation(true);

    SupplyCurrentLimitConfiguration supply{
        true,
        ModuleConstants::kMaxCurrent.value(),
        ModuleConstants::kMaxCurrent.value(),
        10};

    m_topMotor.ConfigSupplyCurrentLimit(supply);
    m_bottomMotor.ConfigSupplyCurrentLimit(supply);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{
            -wpi::numbers::pi},
        units::radian_t(wpi::numbers::pi));

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

// called from DriveSubsystem Periodic
frc::SwerveModuleState DiffSwerveModule::GetState()
{

    double topMotorSpeed = m_topMotor.GetSelectedSensorVelocity();
    double bottomMotorSpeed = m_bottomMotor.GetSelectedSensorVelocity();

    m_driveSpeed = GetDriveSpeed(topMotorSpeed, bottomMotorSpeed);
    m_moduleAngle = GetModuleAngle();

    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/moduleAngle", m_moduleAngle.value());
    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/moduleSpeed", m_driveSpeed.value());

    // data logging
    m_topMotorCurrent.Append(m_topMotor.GetStatorCurrent());
    m_bottomMotorCurrent.Append(m_bottomMotor.GetStatorCurrent());

    m_topMotorRPM.Append(topMotorSpeed);
    m_bottomMotorRPM.Append(bottomMotorSpeed);

    m_wheelSpeed.Append(m_driveSpeed.value());
    m_moduleAngleLog.Append(m_moduleAngle.value());

    return {
        m_driveSpeed,
        frc::Rotation2d(m_moduleAngle)};
}

void DiffSwerveModule::Simulate()
{
    // Simulate the motors
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Top Motor Simulator/Input Voltage", m_topVoltage.value());
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Input Voltage", m_bottomVoltage.value());

    m_topMotorSimulator.SetInputVoltage(m_topVoltage);
    m_bottomMotorSimulator.SetInputVoltage(m_bottomVoltage);

    m_topMotorSimulator.Update(20_ms);
    m_bottomMotorSimulator.Update(20_ms);

    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Top Motor Simulator/Output Velocity", m_topMotorSimulator.GetAngularVelocity().value());
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Output Velocity", m_bottomMotorSimulator.GetAngularVelocity().value());

    // Simulate the intgrated encoders
    m_topMotorSim.SetIntegratedSensorRawPosition((m_topMotorSimulator.GetAngularPosition().value() / (2 * wpi::numbers::pi)) * 2048);
    m_topMotorSim.SetIntegratedSensorVelocity((m_topMotorSimulator.GetAngularVelocity().value() / (2 * wpi::numbers::pi)) * 2048 / 10);

    m_bottomMotorSim.SetIntegratedSensorRawPosition((m_bottomMotorSimulator.GetAngularPosition().value() / (2 * wpi::numbers::pi)) * 2048);
    m_bottomMotorSim.SetIntegratedSensorVelocity((m_bottomMotorSimulator.GetAngularVelocity().value() / (2 * wpi::numbers::pi)) * 2048 / 10);

    // Simulate the encoder
    double averagePos = (m_topMotor.GetSelectedSensorPosition() + m_bottomMotor.GetSelectedSensorPosition()) / 2;
    averagePos /= 2048;
    averagePos /= 28;
    m_encoderSim.Set(units::turn_t{averagePos});
    m_encoderSim.SetDistance(averagePos);

    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Encoder/Rotation", GetModuleAngle().value());
}

units::meters_per_second_t DiffSwerveModule::GetDriveSpeed(double topSpeed, double bottomSpeed)
{
    double speed =
        ((topSpeed - bottomSpeed) / 2) /*Average Sensor Velocity (raw/100ms)*/ *
        (10.0 / 2048) /*Motor Revolutions per second*/ *
        ModuleConstants::kDriveGearRatio /*Output revolutions per second*/ *
        ((ModuleConstants::kDriveWheelDiameterInches / 39.37) * wpi::numbers::pi) /*Circumference in meters (meters/second)*/;

    frc::SmartDashboard::PutNumber(m_name + " Wheel Speed ", speed);

    return units::meters_per_second_t(speed);
}

units::degree_t DiffSwerveModule::GetModuleAngle()
{
    return units::degree_t((m_encoder.Get().value() * 360) - m_offset);
}

units::voltage::volt_t DiffSwerveModule::SetDesiredState(const frc::SwerveModuleState &desiredState)
{

    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize(
        desiredState, m_moduleAngle);

    const units::velocity::meters_per_second_t m_desiredSpeed = state.speed;
    const units::angle::radian_t m_desiredAngle = state.angle.Radians();

    m_expectedSpeed.Append(m_desiredSpeed.value());
    m_expectedAngle.Append(state.angle.Radians().value());

    // Calculate the drive output from the drive PID controller.
    const units::volt_t driveOutput = units::volt_t{
        m_drivePIDController.Calculate(m_driveSpeed.value(), m_desiredSpeed.value())};

    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Drive Speed", m_driveSpeed.value());
    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Desired Speed", m_desiredSpeed.value());
    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Drive Output", driveOutput.value());

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = m_turningPIDController.Calculate(m_moduleAngle, m_desiredAngle);

    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Module Angle", m_moduleAngle.value());
    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Desired Angle", m_desiredAngle.value());
    frc::SmartDashboard::PutNumber("Modules/" + m_name + "/Turn Output", turnOutput);

    // Dont turn at more than 50% power
    turnOutput = std::clamp(turnOutput, -ModuleConstants::kMaxTurnOutput, ModuleConstants::kMaxTurnOutput);

    const units::voltage::volt_t driveFeedForward{
        m_driveFeedForward.Calculate(m_desiredSpeed)};

    m_topVoltage =
        driveOutput + driveFeedForward + DriveConstants::kDriveMaxVoltage * turnOutput;

    m_bottomVoltage = -driveOutput - driveFeedForward + DriveConstants::kDriveMaxVoltage * turnOutput;

    return std::max(m_topVoltage, m_bottomVoltage);
}

void DiffSwerveModule::SetVoltage(units::voltage::volt_t driveMax)
{
    // TODO: replace .Set with .SetVoltage
    m_topMotor.Set(ControlMode::PercentOutput, (m_topVoltage * driveMax / DriveConstants::kDriveMaxVoltage).value());
    m_bottomMotor.Set(ControlMode::PercentOutput, (m_bottomVoltage * driveMax / DriveConstants::kDriveMaxVoltage).value());
}

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

void DiffSwerveModule::SetZeroOffset()
{
    double steerPosition{
        GetModuleAngle().value() + m_offset};
    fmt::print("INFO: {} steerPosition {}\n", m_name, steerPosition);
    frc::Preferences::SetDouble(m_name, steerPosition);
    m_offset = steerPosition;
}

void DiffSwerveModule::LoadZeroOffset()
{
    double steerPosition{
        frc::Preferences::GetDouble(m_name)};
    fmt::print("INFO: {} steerPosition {}\n", m_name, steerPosition);
    m_offset = steerPosition;
}
