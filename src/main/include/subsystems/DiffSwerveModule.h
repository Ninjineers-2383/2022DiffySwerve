#pragma once
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Encoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>
#include <ctre/Phoenix.h>
#include <string>
#include "Constants.h"
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

class DiffSwerveModule
{
    using radians_per_second_squared_t =
        units::compound_unit<
            units::radians,
            units::inverse<units::squared<units::second>>>;

public:
    DiffSwerveModule(int topMotorChannel, int bottomMotorChannel, int encoderPort, std::string name, std::string CANbus, wpi::log::DataLog &log);

    frc::SwerveModuleState GetState();

    double SetDesiredState(const frc::SwerveModuleState &desiredState);

    void ResetEncoders();

    // Set the offset angle for all modules.
    void SetWheelOffset();
    void LoadWheelOffset();

    void MotorsOff();
    // Drive speed in Meters per second
    units::meters_per_second_t GetDriveSpeed(double topSpeed, double bottomSpeed);

    void SetVoltage(double driveMax);

    void Simulate();

    units::degree_t GetModuleAngle();

    wpi::log::DoubleLogEntry m_topMotorCurrent;
    wpi::log::DoubleLogEntry m_bottomMotorCurrent;
    wpi::log::DoubleLogEntry m_topMotorRPM;
    wpi::log::DoubleLogEntry m_bottomMotorRPM;
    wpi::log::DoubleLogEntry m_wheelSpeed;
    wpi::log::DoubleLogEntry m_moduleAngleLog;
    wpi::log::DoubleLogEntry m_expectedSpeed;
    wpi::log::DoubleLogEntry m_expectedAngle;

private:
    static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
        units::radians_per_second_t(wpi::numbers::pi * 100.0); // radians per second

    static constexpr units::radians_per_second_squared_t kModuleMaxAngularAcceleration =
        units::radians_per_second_squared_t(wpi::numbers::pi * 2.0 * 100.0); // radians per second squared

    WPI_TalonFX m_topMotor;
    WPI_TalonFX m_bottomMotor;

    TalonFXSimCollection &m_topMotorSim;
    TalonFXSimCollection &m_bottomMotorSim;

    frc::sim::DCMotorSim m_topMotorSimulator;
    frc::sim::DCMotorSim m_bottomMotorSimulator;

    frc::DutyCycleEncoder m_encoder;

    frc::sim::DutyCycleEncoderSim m_encoderSim;

    std::string m_name;
    wpi::log::DataLog &m_log;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedForward{ModuleConstants::ks, ModuleConstants::kv, ModuleConstants::ka};

    frc2::PIDController m_drivePIDController{
        ModuleConstants::kPModuleDriveController, 0, 0};

    frc::ProfiledPIDController<units::radians> m_turningPIDController{
        ModuleConstants::kPModuleTurningController,
        0.0,
        0.0,
        {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

    double m_offset;

    units::volt_t m_topVoltage;
    units::volt_t m_bottomVoltage;

    units::meters_per_second_t m_driveSpeed;
    units::radian_t m_moduleAngle;
};
