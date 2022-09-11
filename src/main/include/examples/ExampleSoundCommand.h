// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>

#include <frc2/command/CommandHelper.h>

<<<<<<< HEAD:src/main/include/commands/ExampleSoundCommand.h
=======
#include "examples/ExampleSubsystem.h"

>>>>>>> baa43dd5479f7f96cc8b89fc651c364ce1026dba:src/main/include/examples/ExampleSoundCommand.h
#include <networktables/NetworkTableInstance.h>

/**
 * An example command for sending sound cues over network tables
 */
class ExampleSoundCommand
    : public frc2::CommandHelper<frc2::CommandBase, ExampleSoundCommand>
{
public:
    explicit ExampleSoundCommand();

private:
    nt::NetworkTableInstance m_table;
};
