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
    /*
    inline void SetFreq(const float f);
    inline void SetAmp(const float a);
    inline void SetWaveform(const uint8_t wf);
    inline void SetPw(const float pw);
    inline bool IsEOR();
    inline bool IsEOC();
    inline bool IsRising();
    inline bool IsFalling();
    float Process();
    void PhaseAdd(float _phase);
    void Reset(float _phase = 0.0f);
    */
    void SetDetuneAmt(float amt);
    void SetDetuneFreq(float freq);
    void SetRootFreq(float freq);
    float Process();
    void SetWaveshape(float value);
};