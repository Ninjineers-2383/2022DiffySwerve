#include <subsystems/CTREDoubleEncoder.h>

#include <Constants.h>

CTREDoubleEncoder::CTREDoubleEncoder(int portA, int portB, int portAbs)
    : m_quadratureEncoder(portA, portB, false, frc::CounterBase::EncodingType::k4X),
      m_quadratureEncoderSim(m_quadratureEncoder),
      m_absEncoder(portAbs),
      m_absEncoderSim(m_absEncoder)
{
    m_absEncoder.SetPositionOffset(0);
    m_quadratureEncoder.Reset();
    m_quadratureEncoder.SetDistancePerPulse(1);
    ResetEncoders();
}

void CTREDoubleEncoder::ResetEncoders()
{
    m_quadratureEncoder.Reset();
    m_quadratureEncoderSim.SetDistance(0);
    SetZeroOffset();
}

void CTREDoubleEncoder::SetZeroOffset()
{
    zeroOffset = (m_absEncoder.GetAbsolutePosition() * DriveConstants::kEncoderResolution) - m_quadratureEncoder.GetDistance();
}

void CTREDoubleEncoder::Simulate(units::degree_t angle)
{
    m_absEncoderSim.Set(angle);
    m_quadratureEncoderSim.SetDistance(angle.value() / 360 * DriveConstants::kEncoderResolution);
}

units::degree_t CTREDoubleEncoder::Get()
{
    return units::degree_t((m_quadratureEncoder.GetDistance() - zeroOffset) / DriveConstants::kEncoderResolution * 360);
}

int CTREDoubleEncoder::GetRawQuad()
{
    return m_quadratureEncoder.GetDistance();
}

double CTREDoubleEncoder::GetRawAbs()
{
    return m_absEncoder.GetAbsolutePosition();
}