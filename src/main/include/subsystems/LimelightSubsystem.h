#pragma once

#include <frc2/command/SubsystemBase.h>

#include <photonlib/PhotonCamera.h>

class LimelightSubsystem : public frc2::SubsystemBase
{
public:
    LimelightSubsystem();

    void Periodic() override;

    double getX();

    double getY();

    double getLaunchingVelocity();

    bool getTargetVisible();

    void setLimelight(bool isOn);

private:
    photonlib::PhotonCamera camera;

    bool targetValid;

    double targetX;
    double targetY;
};