#include <subsystems/CTREDoubleEncoder.h>

#include <Constants.h>

CTREDoubleEncoder::CTREDoubleEncoder(int portA, int portB, int portAbs)
    : m_quadratureEncoder(portA, portB),
      m_quadratureEncoderSim(m_quadratureEncoder),
      m_absEncoder(portAbs),
      m_absEncoderSim(m_absEncoder)
{
}

void CTREDoubleEncoder::ResetEncoders()
{
    m_quadratureEncoder.Reset();
    m_quadratureEncoderSim.SetDistance(0);
}

void CTREDoubleEncoder::SetZeroOffset()
{
    zeroOffset = int(m_absEncoder.GetAbsolutePosition() * DriveConstants::kEncoderResolution) - m_quadratureEncoder.GetDistance();
}

void CTREDoubleEncoder::Simulate(double angle)
{
    m_absEncoderSim.Set(units::degree_t(angle));
    m_quadratureEncoderSim.SetDistance(angle * DriveConstants::kEncoderResolution);
}

units::degree_t CTREDoubleEncoder::Get()
{
    return units::degree_t((m_quadratureEncoder.GetDistance() - zeroOffset) / DriveConstants::kEncoderResolution * 360);
}

units::degree_t CTREDoubleEncoder::Get(bool simulation)
{
    if (simulation)
    {
        return units::degree_t((m_quadratureEncoder.GetDistance() - zeroOffset) / DriveConstants::kEncoderResolution * 360);
    }
    else
    {
        return units::degree_t(0);
    }
}

int CTREDoubleEncoder::GetRawQuad()
{
    return m_quadratureEncoder.GetDistance();
}

double CTREDoubleEncoder::GetRawAbs()
{
    return m_absEncoder.GetAbsolutePosition();
}