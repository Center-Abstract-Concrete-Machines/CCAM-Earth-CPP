#include <ccam/hw/estuary.h>
#include <ccam/voice/warmosc.h>
#include <ccam/voice/smoothosc.h>
#include <ccam/utils/lockedEstuaryKnobs.h>
#include "daisysp.h"

ccam::hw::Estuary hw;
std::array<WarmOsc, 18> vcos;
std::array<float, 9> detune_constants = {
    0.0f,
    0.16f, 0.25f, 0.38f, 0.55f,
    0.80f, 1.20f, 0.14f, 0.20f
};

std::array<float, 9> octave_constants = {
    1.0f,
    1.0f, 1.0f, 1.0f, 1.0f,
    2.0f, 2.0f, 3.0f, 3.0f
};

SmoothOsc lfo;
daisysp::SmoothRandomGenerator noise;

LockedEstaury main_ctrl;
LockedEstaury left_vco_ctrl;
LockedEstaury right_vco_ctrl;

float combine_cv_safe(float value, uint8_t cv_idx) {
    float cv_in = hw.cvins[cv_idx]->Value();
    // noise
    if (fabs(cv_in) < 0.1f) cv_in = 0.0f;
    return daisysp::fclamp(value + cv_in, 0.0, 1.0f);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
    daisy::AudioHandle::OutputBuffer out, 
    size_t size
) {
    hw.ProcessAllControls();
    main_ctrl.Process();
    left_vco_ctrl.Process();
    right_vco_ctrl.Process();

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

    float left_vco_freq = daisysp::fmap(
        combine_cv_safe(main_ctrl.Value(0), 0),
        10.0f,
        1000.0f
    );
    float right_vco_freq = daisysp::fmap(
        combine_cv_safe(main_ctrl.Value(1), 1),
        10.0f,
        1000.0f
    );
    float amp = combine_cv_safe(main_ctrl.Value(2), 2);
    float detune = combine_cv_safe(main_ctrl.Value(3), 3);

    float lfo_freq = daisysp::fmap(
        main_ctrl.Value(4), 0.1f, 30.0f
    );
    
    lfo.SetFreq(lfo_freq);
    lfo.SetWaveshape(main_ctrl.Value(5));
    noise.SetFreq(
        daisysp::fmap(main_ctrl.Value(6), 0.1f, 30.0f)
    );
    float cv_amp = main_ctrl.Value(7) * 5.0f;

    hw.som.WriteCvOut(1, (lfo.Process() + 1.0f) * cv_amp);
    hw.som.WriteCvOut(2, fabs(noise.Process()) * cv_amp);

    for (size_t v_i = 0; v_i < vcos.size(); v_i++) {

        size_t o_i = v_i % octave_constants.size();

        LockedEstaury* ctrl;
        float freq;
        if (v_i < detune_constants.size()) {
            ctrl = &left_vco_ctrl;
            freq = left_vco_freq;
        } else {
            ctrl = &right_vco_ctrl;
            freq = right_vco_freq;
        }

        if (o_i == 0) {
            vcos[v_i].SetDetuneAmt(0.0f);
            vcos[v_i].SetAmp(amp);
        } else {
            vcos[v_i].SetDetuneAmt(detune);
            vcos[v_i].SetAmp(ctrl->Value(o_i-1) * amp);
        }

        vcos[v_i].SetRootFreq(freq * octave_constants[o_i]);
        vcos[v_i].SetWaveform(wave_type);
    }

    
    LockedEstaury* led_ctrl;
    switch(hw.switches[1].Read()) {
        case daisy::Switch3::POS_LEFT:
            led_ctrl = &main_ctrl;
            break;
        case daisy::Switch3::POS_CENTER:
            led_ctrl = &left_vco_ctrl;
            break;
        case daisy::Switch3::POS_RIGHT:
            led_ctrl = &right_vco_ctrl;
            break;
    }

    for (size_t i = 0; i < hw.leds.size(); i++)
    {
        hw.leds[i].Set(led_ctrl->Value(i) - 0.1);
    }

    for (size_t i = 0; i < size; i++)
    {
        OUT_L[i] = 0.0f;
        OUT_R[i] = 0.0f;
        for (size_t v_i = 0; v_i < octave_constants.size(); v_i++) {
            OUT_L[i] += vcos[v_i].Process();
            OUT_R[i] += vcos[v_i + octave_constants.size()].Process();
        }
    }

    hw.PostProcess();
}

int main(void)
{
    hw.Init();

    // give it a few cycles to initialize the knob values
    // otherwise everything is silent or 100%
    for (size_t i = 0; i < 10; i++) {
        daisy::System::Delay(1);
        hw.ProcessAllControls();
    }

    main_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_LEFT));
    left_vco_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_CENTER));
    right_vco_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_RIGHT));

    for (size_t i = 0; i < vcos.size(); i++) {
        size_t d_i = i % detune_constants.size();
        vcos[i].Init(hw.som.AudioSampleRate());
        vcos[i].SetDetuneFreq(detune_constants[d_i]);
    }

    lfo.Init(hw.som.AudioCallbackRate());
    noise.Init(hw.som.AudioCallbackRate());

    hw.StartAudio(AudioCallback);

    while(1) {
        daisy::System::Delay(1000);
    }
}