#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/TurretSubsystem.h"

class TurretCommand
    : public frc2::CommandHelper<frc2::CommandBase, TurretCommand>
{
public:
    explicit TurretCommand(TurretSubsystem *subsystem, std::function<double()> power, std::function<bool()> seek);

    void Initialize() override;

    void Execute() override;

    void End(bool force) override;

    bool IsFinished() override;

private:
    TurretSubsystem *turret;

    std::function<double()> speed;

    std::function<bool()> seek;

    std::function<bool()> shouldMove;

    bool center;

    bool flipSeek;

    int position;

    bool done;
};