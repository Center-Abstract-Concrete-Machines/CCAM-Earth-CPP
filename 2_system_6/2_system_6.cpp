#include <ccam/hw/estuary.h>
#include <ccam/utils/lockedEstuaryKnobs.h>
#include "daisysp.h"

ccam::hw::Estuary hw;

daisysp::StringVoice string_voice;
daisysp::ModalVoice modal_voice;
LockedEstaury tone_ctrl;
LockedEstaury main_ctrl;

bool string_active = false;
bool modal_active = false;

template <typename Voice>
void set_freq(Voice& voice, uint8_t knob_idx, uint8_t cv_idx) {
    float freq = 0.0f;
    freq += daisysp::fmap(main_ctrl.Value(knob_idx), 1.0f, 1000.f);
    freq *= powf(2.0f, hw.cvins[cv_idx]->Value()*5.0f);
    voice.SetFreq(freq);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
    daisy::AudioHandle::OutputBuffer out, 
    size_t size) {
    hw.ProcessAllControls();
    tone_ctrl.Process();
    main_ctrl.Process();

    // STRING PARAMS
    float string_amp = 0.0f;
    string_amp += main_ctrl.Value(1) * 3.0f;
    string_amp += hw.cvins[1]->Value();
    string_amp = daisysp::fclamp(string_amp, 0.0f, 3.0f);

    float string_perc = main_ctrl.Value(2);
    float string_chance = main_ctrl.Value(3);

    // MODAL PARAMS
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

    if (hw.som.gate_in_1.Trig()) {
        string_active = daisy::Random::GetFloat(0.0f, 1.0f) < string_chance;
        if (string_active) {
            string_voice.Trig();
        }
    }
    if (string_active && hw.som.gate_in_1.State()) {
        set_freq(string_voice, 0, 0);
    }

    modal_voice.SetAccent(tone_ctrl.Value(4));
    modal_voice.SetStructure(tone_ctrl.Value(5));
    modal_voice.SetBrightness(tone_ctrl.Value(6));
    modal_voice.SetDamping(tone_ctrl.Value(7));

    if (hw.som.gate_in_2.Trig()) {
        modal_active = daisy::Random::GetFloat(0.0f, 1.0f) < modal_chance;
        if (modal_active) {
            modal_voice.Trig();
        }
    }
    if (modal_active && hw.som.gate_in_2.State()) {
        set_freq(modal_voice, 4, 2);
    }

    LockedEstaury* led_ctrl;
    switch(hw.switches[1].Read()) {
        case daisy::Switch3::POS_LEFT:
            led_ctrl = &main_ctrl;
            break;
        case daisy::Switch3::POS_CENTER:
        case daisy::Switch3::POS_RIGHT:
            led_ctrl = &tone_ctrl;
            break;
    }

    for (size_t i = 0; i < hw.leds.size(); i++)
    {
        hw.leds[i].Set(led_ctrl->Value(i) - 0.1);
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

    main_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_LEFT));
    tone_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_RIGHT) | (1 << daisy::Switch3::POS_CENTER));

    modal_voice.Init(hw.som.AudioSampleRate());
    string_voice.Init(hw.som.AudioSampleRate());
    hw.StartAudio(AudioCallback);

    hw.som.StartLog(false);
    bool ledOn = false;
    while(1) {
        ledOn = !ledOn;
        hw.leds[0].Set(ledOn ? 0.0f : 1.0f);
        hw.PostProcess();
        daisy::System::Delay(100);
    }
}