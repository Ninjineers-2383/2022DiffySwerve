#include "commands/FlywheelCommand.h"

FlywheelCommand::FlywheelCommand(
    FlywheelSubsystem *subsystem, std::function<double()> speed)
    : flywheel{subsystem},
      m_speed{speed}
{
    AddRequirements(flywheel);
}

void FlywheelCommand::Initialize()
{
}

void FlywheelCommand::Execute()
{
    if (m_shouldChangeSpeed())
    {
        double d_speed = m_speed();
        if (d_speed == previousSpeed)
        {
            return;
        }
        flywheel->Spin(d_speed);
        previousSpeed = d_speed;
    }
}

void FlywheelCommand::End(bool interrupted)
{
}

bool FlywheelCommand::IsFinished()
{
    return false;
}

double FlywheelCommand::speed()
{
    return m_speed();
}

bool FlywheelCommand::launcherRunning()
{
    return m_speed() > 50;
}