#include "subsystems/RetractableIntake.h"

RetractableIntake::RetractableIntake(int motorPort, int solUp, int solDown)
    : m_lift{frc::PneumaticsModuleType::CTREPCM, solUp, solDown},
      m_motor{motorPort}
{
    // Implementation of subsystem constructor goes here.
}

RetractableIntake::RetractableIntake(int motorPort, int solUp, int solDown, bool inactiveWhenRetracted)
    : m_lift{frc::PneumaticsModuleType::CTREPCM, solUp, solDown},
      m_motor{motorPort},
      inactiveWhenRetracted{inactiveWhenRetracted}
{
    // Implementation of subsystem constructor goes here.
}

void RetractableIntake::Periodic()
{
    // Implementation of subsystem periodic method goes here.
}

void RetractableIntake::SimulationPeriodic()
{
    // Implementation of subsystem simulation periodic method goes here.
}

void RetractableIntake::toggleLift(frc::DoubleSolenoid::Value value)
{
    m_lift.Set(value);
}

frc::DoubleSolenoid::Value RetractableIntake::getState()
{
    return m_lift.Get();
}

void RetractableIntake::intake(int power)
{
}
