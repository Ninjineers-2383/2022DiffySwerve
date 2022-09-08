#include "commands/ChimneyCommand.h"

ChimneyCommand::ChimneyCommand(ChimneySubsystem *subsystem, std::function<double()> power)
    : chimney{subsystem},
      power{power},
      previousPower{NAN}
{
    AddRequirements({subsystem});
}

void ChimneyCommand::Execute()
{
    double d_power = power();
    if (d_power == previousPower)
    {
        return;
    }

    chimney->setPower(-d_power);
    previousPower = d_power;
}