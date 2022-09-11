#include "subsystems/IntakeSubsystem.h"

#include <frc/RobotController.h>

#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem(int feederMotorChannel, std::string name, std::string CANbus, wpi::log::DataLog &log)
  :  m_feederMotor{feederMotorChannel},
    m_name(name),
    m_log(log) 
    
{
    m_feederMotor.ConfigFactoryDefault();

    m_feederMotor.SetNeutralMode(NeutralMode::Coast);
}

void IntakeSubsystem::SetPower(double feederPower)
{
    m_feederMotor.Set(ControlMode::PercentOutput, feederPower);
}

void IntakeSubsystem::ZeroPower() 
{
    m_feederMotor.Set(ControlMode::PercentOutput, 0);
}


