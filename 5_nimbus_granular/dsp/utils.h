#ifndef NIMBUS_DSP_UTILS_H_
#define NIMBUS_DSP_UTILS_H_

#include <cstdint>

#define MAKE_INTEGRAL_FRACTIONAL(x)                   \
    int32_t x##_integral   = static_cast<int32_t>(x); \
    float   x##_fractional = x - static_cast<float>(x##_integral);

#define SLOPE(out, in, positive, negative)                \
    {                                                     \
        float error = (in)-out;                           \
        out += (error > 0 ? positive : negative) * error; \
    }

#define ONE_POLE(out, in, coefficient) out += (coefficient) * ((in)-out);

#define CONSTRAIN(var, min, max) \
    if(var < (min))              \
    {                            \
        var = (min);             \
    }                            \
    else if(var > (max))         \
    {                            \
        var = (max);             \
    }

// Simple interpolation function
inline float Interpolate(const float* table, float index, float size)
{
    index *= size;
    MAKE_INTEGRAL_FRACTIONAL(index)
    if (index_integral >= static_cast<int32_t>(size - 1)) {
        return table[static_cast<int32_t>(size - 1)];
    }
    float a = table[index_integral];
    float b = table[index_integral + 1];
    return a + (b - a) * index_fractional;
}

inline float SemitonesToRatio(float semitones)
{
    return powf(2.0f, semitones / 12.0f);
}

inline float Crossfade(float a, float b, float fade)
{
    return a + (b - a) * fade;
}

inline float SoftLimit(float x)
{
    return x / (1.0f + fabsf(x));
}

// Simple random number generation
static uint32_t rng_state = 0x12345678;

inline float Random()
{
    rng_state = rng_state * 1664525 + 1013904223;
    return static_cast<float>(rng_state) / 4294967296.0f;
}

const float kRandFrac = 1.0f / 4294967296.0f;

// Constants
const int32_t kMaxBlockSize = 64;
const int32_t kDownsamplingFactor = 2;

#endif // NIMBUS_DSP_UTILS_H_