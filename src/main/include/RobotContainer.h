#pragma once

#include <frc2/command/Command.h>

#include <frc/Joystick.h>

#include <frc/DataLogManager.h>

#include <frc/DriverStation.h>

#include "commands/JoystickDriveCommand.h"

#include "subsystems/ExampleSubsystem.h"

#include "subsystems/DrivetrainSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the @link Robot periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
  public:
    RobotContainer();

  frc2::Command * GetAutonomousCommand();

  private:
    wpi::log::DataLog & m_log;

  frc::Joystick m_driveJoystick;

  // The robot's subsystems and commands are defined here...
  DrivetrainSubsystem m_drivetrainSubsystem;
  JoystickDriveCommand m_joystickDriveCommand;

  void ConfigureButtonBindings();
  void SetDefaultCommands();
};
