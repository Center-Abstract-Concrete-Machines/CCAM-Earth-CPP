#include <ccam/hw/estuary.h>
#include <ccam/voice/warmosc.h>
#include "daisysp.h"

ccam::hw::Estuary hw;
std::array<WarmOsc, 12> vcos; 
std::array<float, 12> detune_constants = {
    0.0f, 0.16f, 0.25f, 0.38f, 
    0.55f, 0.80f, 1.20f, 0.14f,
    0.20f, 0.30f, 0.45f, 0.65f,
};

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
    daisy::AudioHandle::OutputBuffer out, 
    size_t size
) {
    hw.ProcessAllControls();


    uint8_t wave_type;
    switch(hw.switches[0].Read()) {
        case daisy::Switch3::POS_LEFT:
            wave_type = daisysp::Oscillator::WAVE_SIN;
            break;
        case daisy::Switch3::POS_CENTER:
            wave_type = daisysp::Oscillator::WAVE_SAW;
            break;
        case daisy::Switch3::POS_RIGHT:
            wave_type = daisysp::Oscillator::WAVE_SQUARE;
            break;
    }

    for (WarmOsc& vco : vcos) {
        vco.SetDetuneAmt(hw.knobs[0]->Value());
        vco.SetAmp(hw.knobs[1]->Value());
        vco.SetRootFreq(daisysp::fmap(hw.knobs[2]->Value(), 10.0f, 1000.0f));
        vco.SetWaveform(wave_type);
    }

    size_t num_voices = static_cast<size_t>(
        hw.knobs[3]->Value() * vcos.size()
    );

    for (size_t i = 0; i < size; i++)
    {
        float sum = 0.0f;
        for (size_t voice = 0; voice < vcos.size(); voice++) {
            float value = vcos[voice].Process();
            if (voice < num_voices) {
                sum += value;
            }
        }
        OUT_L[i] = sum;
    }

    hw.PostProcess();
}

int main(void)
{
    hw.Init();

    for (size_t i = 0; i < vcos.size(); i++) {
        vcos[i].Init(hw.som.AudioSampleRate());
        vcos[i].SetDetuneFreq(detune_constants[i]);
    }

    hw.StartAudio(AudioCallback);

    bool ledOn = false;
    while(1) {
        ledOn = !ledOn;
        hw.leds[0].Set(ledOn ? 0.0f : 1.0f);
        daisy::System::Delay(1000);
    }
}