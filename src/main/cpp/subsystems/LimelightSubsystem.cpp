#include "subsystems/LimelightSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <networktables/NetworkTableInstance.h>

#include <photonlib/PhotonPipelineResult.h>

#include <photonlib/PhotonTrackedTarget.h>

#include "Constants.h"

LimelightSubsystem::LimelightSubsystem()
    : camera{"limelight"},
      targetValid(false),
      targetX(0.0),
      targetY(0.0)
{
    camera.SetPipelineIndex(0);
}

void LimelightSubsystem::Periodic()
{
    photonlib::PhotonPipelineResult res = camera.GetLatestResult();

    if (res.HasTargets())
    {
        targetValid = true;
        photonlib::PhotonTrackedTarget bestTarget = res.GetBestTarget();
        targetX = bestTarget.GetYaw();
        targetX = bestTarget.GetPitch();
    }
    else
    {
        targetValid = false;
    }

    frc::SmartDashboard::PutNumber("Target X", targetX);
    frc::SmartDashboard::PutNumber("Target X", targetY);
}

double LimelightSubsystem::getX()
{
    return targetX;
}

double LimelightSubsystem::getY()
{
    return targetY;
}

double LimelightSubsystem::getLaunchingVelocity()
{
    double y = getY();
    return 12801 + (-86.1 * y) + (8.22 * y * y);
}

bool LimelightSubsystem::getTargetVisible()
{
    return targetValid;
}

void LimelightSubsystem::setLimelight(bool isOn)
{
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->GetEntry("ledMode").SetDouble(isOn ? 0 : 2);
}