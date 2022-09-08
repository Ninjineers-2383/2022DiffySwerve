#include "subsystems/KickerSubsystem.h"

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

KickerSubsystem::KickerSubsystem() : indexer{Kicker::PORT}
{
    indexer.SetInverted(false);
    indexer.SetNeutralMode(motorcontrol::Brake);
}

void KickerSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Kicker Velocity", indexer.GetSelectedSensorVelocity());
}

void KickerSubsystem::setPower(double power)
{
    indexer.Set(ControlMode::PercentOutput, power);
}

void KickerSubsystem::setVelocity(double velocity)
{
    indexer.Set(ControlMode::Velocity, velocity);
}