#include <ccam/hw/estuary.h>
#include <ccam/utils/lockedEstuaryKnobs.h>
#include "daisysp.h"

ccam::hw::Estuary hw;

daisysp::StringVoice string_voice;
daisysp::ModalVoice modal_voice;
LockedEstaury tone_ctrl;
LockedEstaury main_ctrl;

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
    daisy::AudioHandle::OutputBuffer out, 
    size_t size) {
    hw.ProcessAllControls();
    tone_ctrl.Process();
    main_ctrl.Process();

    // STRING PARAMS
    float string_freq = 0.0f;
    string_freq += daisysp::fmap(main_ctrl.Value(0), 1.0f, 1000.f);
    string_freq += hw.cvins[0]->Value()*5.0f;
    string_freq = daisysp::fclamp(string_freq, 1.0f, 1000.f);

    float string_amp = 0.0f;
    string_amp += main_ctrl.Value(1) * 3.0f;
    string_amp += hw.cvins[1]->Value();
    string_amp = daisysp::fclamp(string_amp, 0.0f, 3.0f);

    float string_perc = main_ctrl.Value(2);
    float string_chance = main_ctrl.Value(3);

    // MODAL PARAMS
    float modal_freq = 0.0f;
    modal_freq += daisysp::fmap(main_ctrl.Value(4), 1.0f, 1000.f);
    modal_freq += hw.cvins[2]->Value()*5.0f;
    modal_freq = daisysp::fclamp(modal_freq, 1.0f, 1000.f);

    float modal_amp = 0.0f;
    modal_amp += main_ctrl.Value(5) * 3.0f;
    modal_amp += hw.cvins[3]->Value();
    modal_amp = daisysp::fclamp(modal_amp, 0.0f, 3.0f);

    float modal_perc = main_ctrl.Value(6);
    float modal_chance = main_ctrl.Value(7);

    string_voice.SetAccent(tone_ctrl.Value(0));
    string_voice.SetStructure(tone_ctrl.Value(1));
    string_voice.SetBrightness(tone_ctrl.Value(2));
    string_voice.SetDamping(tone_ctrl.Value(3));
    string_voice.SetFreq(string_freq);
    if (hw.som.gate_in_1.Trig() && daisy::Random::GetFloat(0.0f, 1.0f) < string_chance) {
        string_voice.Trig();
    }

    modal_voice.SetAccent(tone_ctrl.Value(4));
    modal_voice.SetStructure(tone_ctrl.Value(5));
    modal_voice.SetBrightness(tone_ctrl.Value(6));
    modal_voice.SetDamping(tone_ctrl.Value(7));
    modal_voice.SetFreq(modal_freq);
    if (hw.som.gate_in_2.Trig() && daisy::Random::GetFloat(0.0f, 1.0f) < modal_chance) {
        modal_voice.Trig();
    }

    for (size_t i = 0; i < size; i++)
    {
        OUT_L[i] = 0.0f;
        OUT_L[i] += (string_voice.Process() * (1.0f - string_perc));
        OUT_L[i] += (string_voice.GetAux() * string_perc);
        OUT_L[i] *= string_amp;
        OUT_L[i] += IN_L[i];
    
        OUT_R[i] = 0.0f;
        OUT_R[i] += (modal_voice.Process() * (1.0f - modal_perc));
        OUT_R[i] += (modal_voice.GetAux() * modal_perc);
        OUT_R[i] *= modal_amp;
        OUT_R[i] += IN_R[i];
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

    tone_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_LEFT));
    main_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_RIGHT) | (1 << daisy::Switch3::POS_CENTER));

    modal_voice.Init(hw.som.AudioSampleRate());
    string_voice.Init(hw.som.AudioSampleRate());
    hw.StartAudio(AudioCallback);

    bool ledOn = false;
    while(1) {
        ledOn = !ledOn;
        hw.leds[0].Set(ledOn ? 0.0f : 1.0f);
        hw.PostProcess();
        daisy::System::Delay(1000);
    }
}