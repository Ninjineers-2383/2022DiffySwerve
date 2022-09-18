#include <subsystems/CTREDoubleEncoder.h>

#include <Constants.h>

CTREDoubleEncoder::CTREDoubleEncoder(int portA, int portB, int portAbs)
    : m_quadratureEncoder(portA, portB, false, frc::CounterBase::EncodingType::k4X), // 4096 ticks per revolution in 4x mode
      m_quadratureEncoderSim(m_quadratureEncoder),
      m_absEncoder(portAbs),
      m_absEncoderSim(m_absEncoder)
{
    ResetEncoders();
}

void CTREDoubleEncoder::ResetEncoders()
{
    m_absEncoder.SetPositionOffset(0);
    m_quadratureEncoder.Reset();
    SetZeroOffset();
}

void CTREDoubleEncoder::SetZeroOffset()
{
    m_zeroOffset =
        (Get().value() + m_zeroOffset)                // Get degrees output from relativeEncoder
        - (m_absEncoder.GetAbsolutePosition() * 360); // Get degrees output from absoluteEncoder
}

void CTREDoubleEncoder::Simulate(units::degree_t angle)
{
    m_absEncoderSim.Set(angle);
    m_quadratureEncoderSim.SetCount(angle.value() / 360 * kEncoderResolution);
}

units::degree_t CTREDoubleEncoder::Get()
{
    return units::degree_t{
        ((m_quadratureEncoder.GetRaw() / kEncoderResolution) // Scale to turns
         * 360)                                              // scale to degrees
        - m_zeroOffset                                       // offset to absolute zero
    };
}

int CTREDoubleEncoder::GetRawQuad()
{
    return m_quadratureEncoder.GetRaw();
}

double CTREDoubleEncoder::GetRawAbs()
{
    return m_absEncoder.GetAbsolutePosition();
}