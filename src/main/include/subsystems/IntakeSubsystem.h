#pragma once

#include <wpi/numbers>

#include <ctre/Phoenix.h>

#include <string>

#include "Constants.h"

#include <frc/DataLogManager.h>

#include <wpi/DataLog.h>

#include <frc/Preferences.h>

class IntakeSubsystem 
{
public:
    IntakeSubsystem(const int feederMotorChannel, 
                    std::string name, std::string CANbus, wpi::log::DataLog &log);

    void SetPower(double feederPower);
    void ZeroPower();

private:
    WPI_VictorSPX m_feederMotor;
    std::string m_name;
    wpi::log::DataLog &m_log;

    double feederPower;
};
