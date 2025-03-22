#pragma once

#include <Synthesis/oscillator.h>
#include <Utility/dsp.h>
#include <array>

class WarmOsc : public daisysp::Oscillator {
    daisysp::Oscillator lfo;
    float detune_amount = 0.0f;
    float root_freq = 0.0f;
public:
    void Init(float sample_rate);
    void SetDetuneAmt(float amt);
    void SetDetuneFreq(float freq);
    void SetRootFreq(float freq);
    float Process();
    void SetWaveshape(float value);
};