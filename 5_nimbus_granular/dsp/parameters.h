#ifndef NIMBUS_DSP_PARAMETERS_H_
#define NIMBUS_DSP_PARAMETERS_H_

struct Parameters
{
    float position;
    float size;
    float pitch;
    float density;
    float texture;
    float dry_wet;
    float stereo_spread;
    float feedback;
    float reverb;

    bool freeze;
    bool trigger;
    bool gate;

    struct Granular
    {
        float overlap;
        float window_shape;
        float stereo_spread;
        bool  use_deterministic_seed;
    } granular;

    struct Spectral
    {
        float quantization;
        float refresh_rate;
        float phase_randomization;
        float warp;
    } spectral;
};

#endif // NIMBUS_DSP_PARAMETERS_H_