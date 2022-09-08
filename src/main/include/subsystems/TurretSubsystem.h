#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix.h>

#include "TurretAssist.h"

using namespace TurretAssist;

class TurretSubsystem : public frc2::SubsystemBase
{
public:
    TurretSubsystem();

    void Periodic() override;

    void setPosition(int pos);

    void coast();

    void brake();

    void setPower(double power);

    void seek();

    void seekDirection(bool direction);

    void runToPosition(int position);

    double getCurrentPosition();

private:
    TalonSRX motor;
    TurretSeekSide boundsState;
};