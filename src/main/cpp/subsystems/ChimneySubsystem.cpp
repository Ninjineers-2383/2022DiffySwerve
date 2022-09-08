#include "subsystems/ChimneySubsystem.h"

#include "Constants.h"

#include <frc/smartdashboard/SmartDashboard.h>

ChimneySubsystem::ChimneySubsystem() : m_motor{ChimneyConstants::Port}
{
    m_motor.SetInverted(true);
    m_motor.SetNeutralMode(NeutralMode::Brake);
}

void ChimneySubsystem::setPower(double power)
{
    m_motor.Set(ControlMode::PercentOutput, power);
    frc::SmartDashboard::PutNumber("Chimney Power", power);
}
