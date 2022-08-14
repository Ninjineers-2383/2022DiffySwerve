#include "subsystems/RetractableIntake.h"

RetractableIntake::RetractableIntake(int motorPort, int solUp, int solDown)
    : m_lift{frc::PneumaticsModuleType::CTREPCM, solUp, solDown},
      m_motor{motorPort}
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
