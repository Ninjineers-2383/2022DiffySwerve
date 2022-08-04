// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>

#include <frc2/command/CommandHelper.h>

#include "subsystems/ExampleSubsystem.h"

#include <networktables/NetworkTableInstance.h>


/**
 * An example command for sending sound cues over network tables
 */
class ExampleSoundCommand
  : public frc2::CommandHelper < frc2::CommandBase, ExampleSoundCommand > {
    public: explicit ExampleSoundCommand();

    private: nt::NetworkTableInstance m_table;
  };
