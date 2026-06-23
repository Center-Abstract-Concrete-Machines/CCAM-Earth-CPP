#ifndef NIMBUS_RESOURCES_H_
#define NIMBUS_RESOURCES_H_

#include <cstdint>

// Simplified resource tables for the granular processor
// Based on Mutable Instruments Clouds but greatly simplified

// Window function for grain envelopes (simple Hann window)
const int32_t kWindowSize = 256;
extern float lut_window[kWindowSize];

// Pitch ratio tables (simplified)
const int32_t kPitchTableSize = 256;
extern float lut_pitch_ratio[kPitchTableSize];

// Crossfade curves
const int32_t kXfadeSize = 256;
extern float lut_xfade_in[kXfadeSize];
extern float lut_xfade_out[kXfadeSize];

// Initialize all lookup tables
void InitResources(float sample_rate);

#endif // NIMBUS_RESOURCES_H_