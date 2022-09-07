#include "subsystems/FlywheelSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/voltage.h>

// Class Declaration
FlywheelSubsystem::FlywheelSubsystem(int flywheelMasterChannel, int flywheelFollowerChannel, std::string name, std::string CANbus, wpi::log::DataLog &log)
    : m_flywheelMaster(flywheelMasterChannel, CANbus),
      m_flywheelFollower(flywheelFollowerChannel, CANbus),
      m_flywheelMasterSim{m_flywheelMaster.GetSimCollection()},
      m_flywheelFollowerSim{m_flywheelFollower.GetSimCollection()},
      m_flywheelMasterSimulator{frc::DCMotor::Falcon500(), 1, units::kilogram_square_meter_t{0.0005}},
      m_flywheelFollowerSimulator{frc::DCMotor::Falcon500(), 1, units::kilogram_square_meter_t{0.0005}},
      m_name(name),
      m_log(log)
{
    /* Actions upon initialization */
    // Reset motors and encoders
    m_flywheelMaster.ConfigFactoryDefault();
    m_flywheelFollower.ConfigFactoryDefault();

    SupplyCurrentLimitConfiguration supply{
        true,
        ModuleConstants::kMaxCurrent.value(),
        ModuleConstants::kMaxCurrent.value(),
        10};

    m_flywheelMaster.ConfigSupplyCurrentLimit(supply);
    m_flywheelFollower.ConfigSupplyCurrentLimit(supply);

    m_flywheelMaster.SetNeutralMode(NeutralMode::Coast);
    m_flywheelFollower.SetNeutralMode(NeutralMode::Coast);

    m_flywheelMasterCurrent = wpi::log::DoubleLogEntry(log, "/" + m_name + "/flywheelMasterCurrent");
    m_flywheelFollowerCurrent = wpi::log::DoubleLogEntry(log, "/" + m_name + "/flywheelFollowerCurrent");

    m_flywheelMasterRPM = wpi::log::DoubleLogEntry(log, "/" + m_name + "/flywheelMasterRPM");
    m_flywheelFollowerRPM = wpi::log::DoubleLogEntry(log, "/" + m_name + "/flywheelFollowerRPM");

    m_wheelSpeed = wpi::log::DoubleLogEntry(log, "/" + m_name + "/wheelSpeed");

    m_expectedSpeed = wpi::log::DoubleLogEntry(log, "/" + m_name + "/expectedSpeed");
}

units::meters_per_second_t FlywheelSubsystem::GetFlywheelSpeed(double topSpeed, double bottomSpeed)
{
    double speed =
        ((topSpeed - bottomSpeed) / 2) /*Average Sensor Velocity (raw/100ms)*/ *
        (10.0 / 2048) /*Motor Revolutions per second*/ *
        ModuleConstants::kDriveGearRatio /*Output revolutions per second*/ *
        ((ModuleConstants::kDriveWheelDiameterInches / 39.37) * wpi::numbers::pi) /*Circumference in meters (meters/second)*/;

    frc::SmartDashboard::PutNumber(m_name + " Wheel Speed ", speed);

    return units::meters_per_second_t(speed);
}

// Periodic function that puts smartdashboard values up
void FlywheelSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Flywheel Master Speed: ", m_flywheelMaster.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Flywheel Follower Speed: ", m_flywheelFollower.GetSelectedSensorVelocity());
}

void FlywheelSubsystem::Spin(double velocity)
{
    if (velocity == 0)
    {
        m_flywheelMaster.Set(ControlMode::PercentOutput, 0);
    }
    else
    {
        m_flywheelMaster.Set(ControlMode::Velocity, velocity);
    }
}

bool FlywheelSubsystem::IsReady()
{
    bool ready = false;
    if (m_flywheelMaster.GetControlMode() == ControlMode::Velocity)
    {
        ready = m_flywheelMaster.GetSelectedSensorVelocity() >= m_flywheelMaster.GetClosedLoopTarget() - FlywheelConstants::Tolerance &&
                m_flywheelMaster.GetSelectedSensorVelocity() <= m_flywheelMaster.GetClosedLoopTarget() + FlywheelConstants::Tolerance;
    }
    frc::SmartDashboard::PutBoolean("Launcher IsReady: ", ready);
    return ready;
}

// Simulation (Brian make this work pls)
void FlywheelSubsystem::Simulate()
{
    // Simulate the motors
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Top Motor Simulator/Input Voltage", m_FlywheelMasterVoltage.value());
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Input Voltage", m_FlywheelFollowerVoltage.value());

    m_flywheelMasterSimulator.SetInputVoltage(m_FlywheelMasterVoltage);
    m_flywheelFollowerSimulator.SetInputVoltage(m_FlywheelFollowerVoltage);

    m_flywheelMasterSimulator.Update(20_ms);
    m_flywheelFollowerSimulator.Update(20_ms);

    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Top Motor Simulator/Output Velocity", m_flywheelMasterSimulator.GetAngularVelocity().value());
    frc::SmartDashboard::PutNumber("Simulated/" + m_name + "/Bottom Motor Simulator/Output Velocity", m_flywheelFollowerSimulator.GetAngularVelocity().value());

    // Simulate the intgrated encoders
    m_flywheelMasterSim.SetIntegratedSensorRawPosition((m_flywheelMasterSimulator.GetAngularPosition().value() / (2 * wpi::numbers::pi)) * 2048);
    m_flywheelMasterSim.SetIntegratedSensorVelocity((m_flywheelMasterSimulator.GetAngularVelocity().value() / (2 * wpi::numbers::pi)) * 2048 / 10);

    m_flywheelFollowerSim.SetIntegratedSensorRawPosition((m_flywheelFollowerSimulator.GetAngularPosition().value() / (2 * wpi::numbers::pi)) * 2048);
    m_flywheelFollowerSim.SetIntegratedSensorVelocity((m_flywheelFollowerSimulator.GetAngularVelocity().value() / (2 * wpi::numbers::pi)) * 2048 / 10);
}
