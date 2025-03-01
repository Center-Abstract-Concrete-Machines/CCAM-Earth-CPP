#include <ccam/hw/estuary.h>
#include "daisysp.h"

ccam::hw::Estuary hw;

daisysp::Metro metro;
std::array<daisysp::Adsr, 2> adsrs;
std::array<daisysp::LadderFilter, 2> vcfs;
std::array<daisy::GateIn*, 2> gates = {
    &hw.som.gate_in_1,
    &hw.som.gate_in_2,
};
std::array<float, 2> out_val;

daisysp::LadderFilter::FilterMode filter_mode_1() {
    switch(hw.switches[0].Read()) {
        case daisy::Switch3::POS_LEFT:
            return daisysp::LadderFilter::FilterMode::LP12;
        case daisy::Switch3::POS_CENTER:
            return daisysp::LadderFilter::FilterMode::BP12;
        case daisy::Switch3::POS_RIGHT:
            return daisysp::LadderFilter::FilterMode::HP12;
    }
    return daisysp::LadderFilter::FilterMode::LP12;
}

daisysp::LadderFilter::FilterMode filter_mode_2() {
    switch(hw.switches[1].Read()) {
        case daisy::Switch3::POS_LEFT:
            return daisysp::LadderFilter::FilterMode::LP24;
        case daisy::Switch3::POS_CENTER:
            return daisysp::LadderFilter::FilterMode::BP24;
        case daisy::Switch3::POS_RIGHT:
            return daisysp::LadderFilter::FilterMode::HP24;
    }
    return daisysp::LadderFilter::FilterMode::LP24;
}

float knob_cv_combo(uint8_t kidx, uint8_t cidx) {
    float value = hw.knobs[kidx]->Value() + hw.cvins[cidx]->Value();
    value -= 0.1; // noise
    return daisysp::fclamp(value, 0.0, 1.0f);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    hw.ProcessAllControls();

    vcfs[0].SetFilterMode(filter_mode_1());
    vcfs[1].SetFilterMode(filter_mode_2());

    for (daisy::Led& led : hw.leds) {
        led.Set(0.0f);
    }

    for (daisysp::Adsr& adsr : adsrs) {
        adsr.SetAttackTime(hw.knobs[0]->Value());
        adsr.SetDecayTime(hw.knobs[1]->Value());
        adsr.SetSustainLevel(hw.knobs[2]->Value());
        adsr.SetReleaseTime(hw.knobs[3]->Value());
    }

    for (uint8_t i = 0; i < vcfs.size(); i++) {
        vcfs[i].SetFreq(daisysp::fmap(knob_cv_combo(4 + i*2, 0 + i*2), 0.1f, 15000.0f));
        vcfs[i].SetRes(knob_cv_combo(5 + i*2, 1 + i*2));
        out_val[i] = adsrs[i].Process(gates[i]->State());
        hw.som.WriteCvOut(i+1, out_val[1-i] * 5.0f);

        uint8_t led = adsrs[i].GetCurrentSegment();
        if (led == 4) { led--; } // who knows
        hw.leds[i*4].Set(out_val[i]);

    }

    for (size_t i = 0; i < size; i++)
    {
        OUT_L[i] = vcfs[0].Process(IN_L[i]);
        OUT_R[i] = vcfs[1].Process(IN_R[i]);
    }

    hw.PostProcess();
}

int main(void)
{
    hw.Init();
    for (daisysp::LadderFilter& vcf : vcfs) {
        vcf.Init(hw.som.AudioSampleRate());
    }
    for (daisysp::Adsr& adsr : adsrs) {
        adsr.Init(hw.som.AudioCallbackRate());
    }
    hw.som.StartLog(false);
    hw.StartAudio(AudioCallback);
    while(1) {
        daisy::System::Delay(1000);
    }
}