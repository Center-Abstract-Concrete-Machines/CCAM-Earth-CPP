#include "resources.h"
#include <cmath>

// Define the lookup tables
float lut_window[kWindowSize];
float lut_pitch_ratio[kPitchTableSize];
float lut_xfade_in[kXfadeSize];
float lut_xfade_out[kXfadeSize];

void InitResources(float sample_rate)
{
    // Initialize Hann window for grain envelopes
    for (int32_t i = 0; i < kWindowSize; i++) {
        float x = static_cast<float>(i) / static_cast<float>(kWindowSize - 1);
        lut_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * x));
    }
    
    // Initialize pitch ratio table (exponential curve)
    for (int32_t i = 0; i < kPitchTableSize; i++) {
        float x = static_cast<float>(i) / static_cast<float>(kPitchTableSize - 1);
        // Map from 0.25x to 4x pitch ratio
        lut_pitch_ratio[i] = powf(2.0f, (x - 0.5f) * 4.0f);
    }
    
    // Initialize crossfade curves
    for (int32_t i = 0; i < kXfadeSize; i++) {
        float x = static_cast<float>(i) / static_cast<float>(kXfadeSize - 1);
        
        // Equal power crossfade
        lut_xfade_in[i] = sinf(x * M_PI * 0.5f);
        lut_xfade_out[i] = cosf(x * M_PI * 0.5f);
    }
}