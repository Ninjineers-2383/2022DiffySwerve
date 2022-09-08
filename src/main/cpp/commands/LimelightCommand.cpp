#include "commands/LimelightCommand.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/MathUtil.h>

#include "Constants.h"

LimelightCommand::LimelightCommand(LimelightSubsystem *subsystem)
    : limelight{subsystem},
      turretPower{0},
      turretSeek{false},
      lockedOn{false},
      limelightF{5},
      driveVelocity{0},
      turretTicks{0}
{
    AddRequirements(subsystem);
}

LimelightCommand::LimelightCommand(LimelightSubsystem *subsystem, std::function<double()> turretTicks, std::function<double()> driveTrainVelocity)
    : limelight{subsystem},
      turretPower{0},
      turretSeek{false},
      lockedOn{false},
      limelightF{5},
      driveVelocity{driveTrainVelocity},
      turretTicks{turretTicks}
{
    AddRequirements(subsystem);
}

void LimelightCommand::Execute()
{
    frc::SmartDashboard::PutBoolean("Locked On", lockedOn);

    turretSeek = false;

    double error = limelightF.Calculate(limelight->getX());
    lockedOn = error < 3 && limelight->getTargetVisible();
    if (limelight->getTargetVisible())
    {
        limelight->setLimelight(true);
        std::clamp(((std::abs(error) > 0.4) ? 1 : 0) * (Turret::kP * error), -Turret::ADJUST_POWER, Turret::ADJUST_POWER);
    }
    else
    {
        turretSeek = true;
    }
}

double LimelightCommand::getTurretPower()
{
    return turretPower;
}

bool LimelightCommand::getTurretSeek()
{
    return turretSeek;
}

bool LimelightCommand::getLockedOn()
{
    return lockedOn;
}

bool LimelightCommand::getTurretRunning()
{
    return turretPower > 0.05;
}