#pragma once

#include <frc/Encoder.h>

#include <frc/simulation/EncoderSim.h>

#include <frc/DutyCycleEncoder.h>

#include <frc/simulation/DutyCycleEncoderSim.h>

class CTREDoubleEncoder
{
public:
    CTREDoubleEncoder(const int portA, const int portB, const int portAbs);

    void ResetEncoders();
    void SetZeroOffset();
    void Simulate(units::degree_t angle);

    units::degree_t Get();

    int GetRawQuad();
    double GetRawAbs();

private:
    double zeroOffset = 0;
    frc::Encoder m_quadratureEncoder;
    frc::sim::EncoderSim m_quadratureEncoderSim;
    frc::DutyCycleEncoder m_absEncoder;
    frc::sim::DutyCycleEncoderSim m_absEncoderSim;
};