// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ExampleSoundCommand.h"

ExampleSoundCommand::ExampleSoundCommand()
    : m_table(nt::NetworkTableInstance::GetDefault())
{
    if (false)
    {
        m_table.GetTable("SoundCues")->PutString("PlayDriverCue", "");
        m_table.GetTable("SoundCues")->PutString("PlayDriverCue", "Drive");
    }
    if (false)
    {
        m_table.GetTable("SoundCues")->PutString("PlayOperatorCue", "");
        m_table.GetTable("SoundCues")->PutString("PlayOperatorCue", "Shoot");
    }
}
