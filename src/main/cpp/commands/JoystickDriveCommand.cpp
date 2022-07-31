#include "commands/JoystickDriveCommand.h"

JoystickDriveCommand::JoystickDriveCommand(DrivetrainSubsystem *drivetrain, frc::Joystick *joystick)
    : m_drivetrain(drivetrain),
      m_joystick(joystick),
      m_table(nt::NetworkTableInstance::GetDefault())
{
    AddRequirements(m_drivetrain);
}

void JoystickDriveCommand::Execute()
{
    auto xAxis = -frc::ApplyDeadband(m_joystick->GetY(), 0.1) * DriveConstants::kMaxSpeed;
    auto yAxis = -frc::ApplyDeadband(m_joystick->GetX(), 0.1) * DriveConstants::kMaxSpeed;
    auto zAxis = -frc::ApplyDeadband(m_joystick->GetZ(), 0.1) * DriveConstants::kMaxSpeed;

    m_drivetrain->SetFieldCentric(!m_joystick->GetRawButton(3)); // Set Field Centered while not holding 3 button

    if (m_joystick->GetRawButtonPressed(1))
    {
        m_table.GetTable("SoundCues")->PutString("PlayDriverCue", "");
        m_table.GetTable("SoundCues")->PutString("PlayDriverCue", "Drive");
    }
    if (m_joystick->GetRawButtonPressed(2))
    {
        m_table.GetTable("SoundCues")->PutString("PlayOperatorCue", "");
        m_table.GetTable("SoundCues")->PutString("PlayOperatorCue", "Shoot");
    }

    m_drivetrain->Drive(xAxis, yAxis, units::radians_per_second_t(zAxis.value()));
}

void JoystickDriveCommand::End(bool interrupted)
{
    m_drivetrain->MotorsOff();
}

bool JoystickDriveCommand::IsFinished()
{
    return false;
}