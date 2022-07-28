#include "commands/JoystickDriveCommand.h"

JoystickDriveCommand::JoystickDriveCommand(DrivetrainSubsystem *drivetrain, frc::Joystick *joystick)
    : m_drivetrain(drivetrain),
      m_joystick(joystick)
{
    AddRequirements(m_drivetrain);
}

void JoystickDriveCommand::Execute()
{
    auto xAxis = -frc::ApplyDeadband(m_joystick->GetX(), 0.1) * DriveConstants::kMaxSpeed;
    auto yAxis = -frc::ApplyDeadband(m_joystick->GetY(), 0.1) * DriveConstants::kMaxSpeed;
    auto zAxis = frc::ApplyDeadband(m_joystick->GetZ(), 0.1) * DriveConstants::kMaxSpeed / 2;

    m_drivetrain->SetFieldCentric(m_joystick->GetTrigger());

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