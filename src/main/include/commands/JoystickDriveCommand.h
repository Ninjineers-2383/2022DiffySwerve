// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>

#include "subsystems/DrivetrainSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class JoystickDriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, JoystickDriveCommand>
{
public:
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit JoystickDriveCommand(DrivetrainSubsystem *drivetrain, frc::Joystick *joystick);

    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    DrivetrainSubsystem *m_drivetrain;
    frc::Joystick *m_joystick;
};
