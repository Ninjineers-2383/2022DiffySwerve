#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/DoubleSolenoid.h>

#include <ctre/Phoenix.h>

class RetractableIntake : public frc2::SubsystemBase
{
public:
    /**
     * Retractable motorized intake
     * @param motorPort can ID for motor
     * @param solUp pcm port num for up actuation
     * @param solDown pcm port num for down actuation
     */
    RetractableIntake(int motorPort, int solUp, int solDown);
    RetractableIntake(int motorPort, int solUp, int solDown, bool inactiveWhenRetracted);
    void RetractableIntake::toggleLift(frc::DoubleSolenoid::Value value);
    frc::DoubleSolenoid::Value RetractableIntake::getState();
    void RetractableIntake::intake(int power);
    frc::DoubleSolenoid m_lift;
    WPI_TalonFX m_motor;
    bool RetractableIntake::inactiveWhenRetracted;
    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void
    Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};