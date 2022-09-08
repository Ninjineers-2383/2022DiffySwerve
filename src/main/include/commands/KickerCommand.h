#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/KickerSubsystem.h"

class KickerCommand
    : public frc2::CommandHelper<frc2::CommandBase, KickerCommand>
{
public:
    explicit KickerCommand(KickerSubsystem *subsystem, std::function<double()> power);

    void Execute() override;

    void End(bool interrupted) override;

private:
    KickerSubsystem *indexer;

    std::function<double()> power;

    double previousPower;
};