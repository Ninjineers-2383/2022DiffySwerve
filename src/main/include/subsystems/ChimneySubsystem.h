#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix.h>

class ChimneySubsystem : public frc2::SubsystemBase
{
public:
    ChimneySubsystem();

    void setPower(double power);

private:
    WPI_VictorSPX m_motor;
};