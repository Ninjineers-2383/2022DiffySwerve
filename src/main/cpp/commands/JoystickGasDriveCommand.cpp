#include "commands/JoystickGasDriveCommand.h"
#include <math.h>

JoystickGasDriveCommand::JoystickGasDriveCommand(DrivetrainSubsystem *drivetrain, std::function<double()> wInput, std::function<double()> xInput,
                                                 std::function<double()> yInput, std::function<double()> zInput,
                                                 std::function<bool()> fieldCentricToggle)
    : m_drivetrain(drivetrain),
      m_wInput(wInput),
      m_xInput(xInput),
      m_yInput(yInput),
      m_zInput(zInput),
      m_fieldCentricToggle(fieldCentricToggle)
{
    AddRequirements(m_drivetrain);
}

void JoystickGasDriveCommand::Execute()
{
    double angle = atan2(throttleSoftener(frc::ApplyDeadband(m_yInput(), 0.1)), throttleSoftener(frc::ApplyDeadband(m_xInput(), 0.1)));

    units::velocity::meters_per_second_t xAxis = -m_wInput() * cos(angle) * DriveConstants::kMaxSpeed;
    units::velocity::meters_per_second_t yAxis = -m_wInput() * sin(angle) * DriveConstants::kMaxSpeed;
    units::velocity::meters_per_second_t zAxis = -throttleSoftener(frc::ApplyDeadband(m_zInput(), 0.1)) * DriveConstants::kMaxSpeed;

    m_drivetrain->SetFieldCentric(m_fieldCentricToggle());

    m_drivetrain->Drive(xAxis, yAxis, units::radians_per_second_t(zAxis.value()));
}

void JoystickGasDriveCommand::End(bool interrupted)
{
    m_drivetrain->MotorsOff();
}

bool JoystickGasDriveCommand::IsFinished()
{
    return false;
}

double throttleSoftener(double input)
{
    constexpr double gain = 0.5; // Minimum gain is -0.5 before things start doing weird things
    double result = gain * (input * input * input) + (1 - gain) * input;
    return result;
}