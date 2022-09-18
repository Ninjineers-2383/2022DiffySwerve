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

#include "subsystems/CTREDoubleEncoder.h"

class DiffSwerveModule : public wpi::Sendable
{
    using radians_per_second_squared_t =
        units::compound_unit<
            units::radians,
            units::inverse<units::squared<units::second>>>;

public:
    /**
     * Differential Swerve Module
     *
     * @param topMotorChannel can ID of top falcon
     * @param bottomMotorChannel can ID of bottom falcon
     * @param encoderPortA port number of the A channel of the quadrature encoder
     * @param encoderPortB port number of the B channel of the quadrature encoder
     * @param encoderPortAbs port number of the DutyCycle channel of the absolute encoder
     * @param name string for module name
     * @param CANbus string for CANbus
     * @param log wpi datalog
     */
    DiffSwerveModule(const int topMotorChannel, const int bottomMotorChannel,
                     const int encoderPortA, const int encoderPortB, const int encoderPortAbs,
                     std::string name, std::string CANbus, wpi::log::DataLog &log);

    /**
     * @return state of module
     */
    frc::SwerveModuleState GetState();

    /**
     * Set state of module
     * @param desiredState target state
     */
    units::voltage::volt_t SetDesiredState(const frc::SwerveModuleState &desiredState);

    /* Reset encoders */
    void ResetEncoders();

    /* Set the offset angle */
    void SetZeroOffset();

    /* Load offset angle */
    void LoadZeroOffset();

    /* Turn off both motors */
    void MotorsOff();

    /* Get drive speed */
    units::meters_per_second_t GetDriveSpeed(double topSpeed, double bottomSpeed);

    /* Update motor powers */
    void SetVoltage(units::voltage::volt_t driveMax);

    /* Simulate modules */
    void Simulate();

    /**
     *  @return Module angle
     */
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
    void InitSendable(wpi::SendableBuilder &builder) override;

    WPI_TalonFX m_topMotor;
    WPI_TalonFX m_bottomMotor;

    TalonFXSimCollection &m_topMotorSim;
    TalonFXSimCollection &m_bottomMotorSim;

    frc::sim::DCMotorSim m_topMotorSimulator;
    frc::sim::DCMotorSim m_bottomMotorSimulator;

    CTREDoubleEncoder m_encoder;

    std::string m_name;
    wpi::log::DataLog &m_log;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedForward{
        ModuleConstants::ks, ModuleConstants::kv, ModuleConstants::ka};

    frc2::PIDController m_drivePIDController{
        ModuleConstants::kPModuleDriveController, 0, 0};

    frc::ProfiledPIDController<units::radians> m_turningPIDController{
        ModuleConstants::kPModuleTurningController,
        0.0,
        0.0,
        {ModuleConstants::kMaxAngularVelocity,
         ModuleConstants::kMaxAngularAcceleration}};

    units::degree_t m_offset;

    units::volt_t m_topVoltage;
    units::volt_t m_bottomVoltage;

    units::meters_per_second_t m_driveSpeed;
    units::radian_t m_moduleAngle;

    units::meters_per_second_t m_desiredSpeed;
    units::radian_t m_desiredAngle;

    units::volt_t m_driveOutput;
    double m_turnOutput;
};
