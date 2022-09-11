#include "helpers/ThrottleSoftener.h"

double ThrottleSoftener(double input)
{
    constexpr double gain = 0.5; // Minimum gain is -0.5 before things start doing weird things
    double result = gain * (input * input * input) + (1 - gain) * input;
    return result;
};