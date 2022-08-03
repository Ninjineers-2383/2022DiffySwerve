#include "commands/JoystickDriveCommand.h"

JoystickDriveCommand::JoystickDriveCommand(DrivetrainSubsystem *drivetrain, std::function<double()> xInput, std::function<double()> yInput, std::function<double()> zInput, std::function<bool()> fieldCentricToggle)
    : m_drivetrain(drivetrain),
      m_xInput(xInput),
      m_yInput(yInput),
      m_zInput(zInput),
      m_fieldCentricToggle(fieldCentricToggle) 
{
    AddRequirements(m_drivetrain);
}

void JoystickDriveCommand::Execute()
{
    auto xAxis = -frc::ApplyDeadband(m_xInput(), 0.1) * DriveConstants::kMaxSpeed;
    auto yAxis = -frc::ApplyDeadband(m_yInput(), 0.1) * DriveConstants::kMaxSpeed;
    auto zAxis = -frc::ApplyDeadband(m_zInput(), 0.1) * DriveConstants::kMaxSpeed;

    m_drivetrain->SetFieldCentric(m_fieldCentricToggle()); // Set Field Centered while not holding 3 button

    //TODO: fix this mess because IDK what else to do

    

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