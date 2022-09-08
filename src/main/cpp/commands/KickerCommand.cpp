#include "commands/KickerCommand.h"

KickerCommand::KickerCommand(KickerSubsystem *subsystem, std::function<double()> power)
    : indexer{subsystem},
      power{power},
      previousPower{NAN}
{
    AddRequirements(subsystem);
}

void KickerCommand::Execute()
{
    double d_power = power();
    if (d_power == previousPower)
    {
        return;
    }
    indexer->setPower(d_power);
    previousPower = d_power;
}

void KickerCommand::End(bool interrupted)
{
}