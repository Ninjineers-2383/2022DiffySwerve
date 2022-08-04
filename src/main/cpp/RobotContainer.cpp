#include "RobotContainer.h"

RobotContainer::RobotContainer(): m_log {
    frc::DataLogManager::GetLog()
  },
  m_driveJoystick {
    0
  },
  m_drivetrainSubsystem {
    m_log
  },
  m_joystickDriveCommand {
    &
    m_drivetrainSubsystem, [ & ]() {
      return m_driveJoystick.GetX();
    },
    [ & ]() {
      return m_driveJoystick.GetY();
    },
    [ & ]() {
      return m_driveJoystick.GetZ();
    },
    [ & ]() {
      return m_driveJoystick.GetRawButtonPressed(1);
    }
  } {
    frc::DataLogManager::Start();

    frc::DriverStation::StartDataLog(m_log);

    SetDefaultCommands();

    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    ConfigureButtonBindings();
  }

void RobotContainer::SetDefaultCommands() {
  m_drivetrainSubsystem.SetDefaultCommand(m_joystickDriveCommand);
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command * RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
