#pragma once

#include <frc/controller/PIDController.h>

#include <frc/controller/ProfiledPIDController.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/Encoder.h>

#include <frc/simulation/DCMotorSim.h>

#include <frc/motorcontrol/Spark.h>

#include <frc/Preferences.h>

#include <frc/trajectory/TrapezoidProfile.h>

#include <wpi/numbers>

#include <ctre/Phoenix.h>

#include <string>

#include "Constants.h"

#include <frc/DataLogManager.h>

#include <wpi/DataLog.h>

#include <frc2/command/SubsystemBase.h>

class FlywheelSubsystem : frc2::SubsystemBase
{
public:
    FlywheelSubsystem(std::string name, std::string CANbus, wpi::log::DataLog &log);
    /*
     * Differential Swerve Module
     *
     * @param flywheelMasterPort can ID of master flywheel falcon
     * @param flywheelFollowerPort can ID of follower flywheel falcon
     * @param name string for module name
     * @param CANbus string for CANbus
     * @param log wpi datalog
     */
    /**
     * Set state of module
     * @param desiredState
     */
    /* Periodic */
    void Periodic() override;

    /* Flywheel spin */
    void Spin(double velocity);

    /* public boolean to see if flywheel is running */
    bool IsReady();

    /* Get drive speed */
    units::meters_per_second_t GetFlywheelSpeed(double FlywheelMasterSpeed, double FlywheelFollowerSpeed);

    /* Simulate modules */
    void Simulate();

    wpi::log::DoubleLogEntry m_flywheelMasterCurrent;
    wpi::log::DoubleLogEntry m_flywheelFollowerCurrent;
    wpi::log::DoubleLogEntry m_flywheelMasterRPM;
    wpi::log::DoubleLogEntry m_flywheelFollowerRPM;
    wpi::log::DoubleLogEntry m_wheelSpeed;
    wpi::log::DoubleLogEntry m_moduleAngleLog;
    wpi::log::DoubleLogEntry m_expectedSpeed;
    WPI_TalonFX m_flywheelMaster;
    WPI_TalonFX m_flywheelFollower;

    TalonFXSimCollection &m_flywheelMasterSim;
    TalonFXSimCollection &m_flywheelFollowerSim;

    frc::sim::DCMotorSim m_flywheelMasterSimulator;
    frc::sim::DCMotorSim m_flywheelFollowerSimulator;

    std::string m_name;
    wpi::log::DataLog &m_log;

    frc::SimpleMotorFeedforward<units::meters> m_FlywheelFeedForward{
        ModuleConstants::ks, ModuleConstants::kv, ModuleConstants::ka};

    units::volt_t m_FlywheelMasterVoltage;
    units::volt_t m_FlywheelFollowerVoltage;

    units::meters_per_second_t m_FlywheelSpeed;
};
