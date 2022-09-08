#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/filter/MedianFilter.h>

#include "subsystems/LimelightSubsystem.h"

class LimelightCommand
    : public frc2::CommandHelper<frc2::CommandBase, LimelightCommand>
{
public:
    explicit LimelightCommand(LimelightSubsystem *subsystem);

    explicit LimelightCommand(LimelightSubsystem *subsystem, std::function<double()> turretTicks, std::function<double()> driveTrainVelocity);

    void Execute() override;

    double getTurretPower();

    bool getTurretSeek();

    bool getLockedOn();

    bool getTurretRunning();

private:
    LimelightSubsystem *limelight;

    double turretPower;

    bool turretSeek;

    bool lockedOn;

    frc::MedianFilter<size_t> limelightF;

    std::function<double()> driveVelocity;

    std::function<double()> turretTicks;
};