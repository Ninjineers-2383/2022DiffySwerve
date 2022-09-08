#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ChimneySubsystem.h"

class ChimneyCommand
    : public frc2::CommandHelper<frc2::CommandBase, ChimneyCommand>
{
public:
    explicit ChimneyCommand(ChimneySubsystem *subsystem, std::function<double()> power);

    void Execute() override;

private:
    ChimneySubsystem *chimney;

    std::function<double()> power;
    double previousPower;
};