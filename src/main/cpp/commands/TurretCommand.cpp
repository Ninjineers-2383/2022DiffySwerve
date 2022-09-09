#include "commands/TurretCommand.h"

TurretCommand::TurretCommand(TurretSubsystem *subsystem, std::function<double()> power, std::function<bool()> seek)
    : turret{subsystem},
      speed{power},
      seek{seek},
      center{false},
      position{position}
{
    AddRequirements(turret);
}

void TurretCommand::Initialize()
{
    turret->seekDirection(flipSeek);
}

void TurretCommand::Execute()
{
    if (shouldMove())
    {
        if (center)
        {
            if (std::abs(turret->getCurrentPosition() - position) > 300)
            {
                turret->runToPosition(position);
            }
            else
            {
                turret->setPower(0.0);
                done = false;
            }
        }
        else
        {
            if (seek())
            {
                turret->seek();
            }
            else
            {
                turret->setPower(speed());
            }
        }
    }
    else
    {
        turret->setPower(speed());
    }
}

void TurretCommand::End(bool force)
{
    turret->setPower(0.0);
}

bool TurretCommand::IsFinished()
{
    return done;
}