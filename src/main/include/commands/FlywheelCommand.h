#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Timer.h>

#include "subsystems/FlywheelSubsystem.h"

class FlywheelCommand
    : public frc2::CommandHelper<frc2::CommandBase, FlywheelCommand>
{
public:
    explicit FlywheelCommand(FlywheelSubsystem *subsystem, std::function<double()> speed);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    double speed();

    bool launcherRunning();

private:
    FlywheelSubsystem *flywheel;

    std::function<double()> m_speed;

    std::function<bool()> m_shouldChangeSpeed;

    double previousSpeed;

    frc::Timer timer;
};