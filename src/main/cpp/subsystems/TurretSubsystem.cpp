#include "subsystems/TurretSubsystem.h"

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

TurretSubsystem::TurretSubsystem() : motor{Turret::PORT}, boundsState{TurretSeekSide::kForward}
{
    motor.SetInverted(false);
    motor.SetSensorPhase(true);
    motor.SetSelectedSensorPosition(0);
    brake();
}

void TurretSubsystem::Periodic()
{
    frc::SmartDashboard::PutNumber("Turret Pos", getCurrentPosition());
}

void TurretSubsystem::setPosition(int pos)
{
    motor.SetSelectedSensorPosition(pos);
}

void TurretSubsystem::coast()
{
    motor.SetNeutralMode(NeutralMode::Coast);
}
void TurretSubsystem::brake()
{
    motor.SetNeutralMode(NeutralMode::Brake);
}

void TurretSubsystem::setPower(double power)
{
    if (getCurrentPosition() > Turret::BOUNDS)
    {
        power = -500.0;
        boundsState = TurretSeekSide::kReverse;
    }
    else if (getCurrentPosition() < -Turret::BOUNDS)
    {
        power = 500.0;
        boundsState = TurretSeekSide::kForward;
    }
    motor.Set(motorcontrol::ControlMode::Velocity, power);
    frc::SmartDashboard::PutNumber("446pm", power);
    if (boundsState == TurretSeekSide::kForward)
    {
        frc::SmartDashboard::PutString("Side", "Over Bounds");
    }
    else
    {
        frc::SmartDashboard::PutString("Side", "Under Bounds");
    }
}

void TurretSubsystem::seek()
{
    setPower(boundsState == TurretSeekSide::kForward ? Turret::SEEKING_POWER : -Turret::SEEKING_POWER);
}

void TurretSubsystem::seekDirection(bool direction)
{
    if (direction)
    {
        boundsState = TurretSeekSide::kForward;
    }
    else
    {
        boundsState = TurretSeekSide::kReverse;
    }
}

void TurretSubsystem::runToPosition(int position)
{
    double error = -getCurrentPosition() - position;
    if (abs(error) > 100)
    {
        setPower(Turret::kP_CENTER * error);
    }
    else
    {
        setPower(0.0);
    }
}

double TurretSubsystem::getCurrentPosition()
{
    return motor.GetSelectedSensorPosition(0);
}