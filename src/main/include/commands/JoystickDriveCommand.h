#pragma once

#include <frc2/command/CommandBase.h>

#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>

#include "subsystems/DrivetrainSubsystem.h"

#include <functional>

/**
 * Joystick drive command responsible for Teleop control of the drivetrain subsystem
 */
class JoystickDriveCommand
  : public frc2::CommandHelper < frc2::CommandBase, JoystickDriveCommand > {
    public:
      /**
       * Joystick drive command responsible for Teleop control of the drivetrain subsystem
       *
       * @param drivetrain Drivetrain subsystem
       * @param xInput Input for x motion
       * @param yInput Input for y motion
       * @param zInput Input for rotation
       * @param fieldCentricToggle Input for disabling field-centric
       */
      explicit JoystickDriveCommand(DrivetrainSubsystem * drivetrain, std:: function < double() > xInput, std:: function < double() > yInput, std:: function < double() > zInput, std:: function < bool() > fieldCentricToggle);

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

    private: DrivetrainSubsystem * m_drivetrain;
    std:: function < double() > m_xInput;
    std:: function < double() > m_yInput;
    std:: function < double() > m_zInput;
    std:: function < bool() > m_fieldCentricToggle;
  };
  