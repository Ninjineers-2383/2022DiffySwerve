#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix.h>

class KickerSubsystem : public frc2::SubsystemBase
{
public:
    KickerSubsystem();

    void Periodic() override;

    void setPower(double power);

    void setVelocity(double velocity);

private:
    WPI_VictorSPX indexer;
};