#include "warmosc.h"

void WarmOsc::Init(float sample_rate) {
    lfo.Init(sample_rate);
    daisysp::Oscillator::Init(sample_rate);
}

void WarmOsc::SetDetuneAmt(float amt) {
    detune_amount = amt;
}

void WarmOsc::SetDetuneFreq(float freq) {
    lfo.SetFreq(freq);
}

void WarmOsc::SetRootFreq(float freq) {
    root_freq = freq;
}

float WarmOsc::Process() {
    SetFreq(root_freq * (1.0f + (lfo.Process() * detune_amount)));
    return daisysp::Oscillator::Process();
}