#pragma once

#include <frc/Encoder.h>

#include <frc/simulation/EncoderSim.h>

#include <frc/DutyCycleEncoder.h>

#include <frc/simulation/DutyCycleEncoderSim.h>

class CTREDoubleEncoder
{
public:
    /**
     * Differential Swerve Module
     *
     * @param portA DIO pin number of quadrature channel A
     * @param portB DIO pin number of quadrature channel B
     * @param portAbs port number of the DutyCycle channel of the absolute encoder
     */
    CTREDoubleEncoder(const int portA, const int portB, const int portAbs);

    // Reset quadrature encoder
    void ResetEncoders();

    // Set quadrature offset with absolute encoder
    void SetZeroOffset();

    // Simulation support
    void Simulate(units::degree_t angle);

    /**
     * @return adjusted degrees from absolute zero
     */
    units::degree_t Get();

    /**
     * @return raw quadrature encoder ticks
     */
    int GetRawQuad();

    /**
     * @return raw absolute encoder reading
     */
    double GetRawAbs();

private:
    double zeroOffset = 0;
    frc::Encoder m_quadratureEncoder;
    frc::sim::EncoderSim m_quadratureEncoderSim;
    frc::DutyCycleEncoder m_absEncoder;
    frc::sim::DutyCycleEncoderSim m_absEncoderSim;
};