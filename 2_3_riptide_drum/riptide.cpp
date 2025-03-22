#include <ccam/hw/estuary.h>
#include <ccam/voice/tonedrum.h>
#include <ccam/voice/noisedrum.h>
#include <ccam/utils/lockedEstuaryKnobs.h>
#include <ccam/utils/gubbins.h>
#include <ccam/seq/gridseq.h>
#include <ccam/utils/gateclock.h>

#include "daisysp.h"

bool ledOn = false;

ccam::hw::Estuary hw;

GateClock<LockedAnalogControl> clock;

std::array<grids::PatternGenerator, 2> patgens;

ToneDrum tone_drum;
NoiseDrum noise_drum;

using LockedCvKnob = CvKnob<LockedAnalogControl, daisy::AnalogControl>;

LockedEstaury knobs_vco;
LockedEstaury knobs_seq;

LockedCvKnob tone_decay;
LockedCvKnob tone_freq;
LockedCvKnob fm_amount;
LockedCvKnob fm_decay;

LockedCvKnob noise_decay;
LockedCvKnob noise_freq;

LockedCvKnob tone_amp;
LockedCvKnob noise_amp;

LockedCvKnob x_ctrl;
LockedCvKnob y_ctrl;
LockedCvKnob chaos;

LockedCvKnob tone_fill;
LockedCvKnob noise_fill;

std::array<LockedCvKnob*, 14> cvknobs = {
    &tone_decay, &tone_freq, &fm_amount, &fm_decay,
    &noise_decay, &noise_freq, &tone_amp, &noise_amp,
    &x_ctrl, &y_ctrl, &chaos, &tone_fill, &noise_fill
};

std::array<bool, 2> gates;

void Process() {
    for (uint8_t channel = 0; channel < patgens.size(); channel++) {
        patgens[channel].Tick();
        gates[channel] = patgens[channel].Triggered();
    }
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    hw.ProcessAllControls();
    knobs_vco.Process();
    knobs_seq.Process();

    for (LockedCvKnob* cvknob : cvknobs) {
        cvknob->cvin = nullptr;
    }

    switch (hw.switches[0].Read()) {
        case daisy::Switch3::POS_LEFT:
            tone_decay.cvin = hw.cvins[0];
            tone_freq.cvin = hw.cvins[1];
            noise_decay.cvin = hw.cvins[2];
            noise_freq.cvin = hw.cvins[3];
            break;
        case daisy::Switch3::POS_CENTER:
            x_ctrl.cvin = hw.cvins[0];
            chaos.cvin = hw.cvins[1];
            tone_fill.cvin = hw.cvins[2];
            noise_fill.cvin = hw.cvins[3];
            break;
        case daisy::Switch3::POS_RIGHT:
            tone_decay.cvin = hw.cvins[0];
            noise_decay.cvin = hw.cvins[1];
            tone_fill.cvin = hw.cvins[2];
            noise_fill.cvin = hw.cvins[3];
            break;
    }

    tone_drum.SetFreq(daisysp::fmap(tone_freq.Value(), 5.0f, 880.0f));
    tone_drum.SetLength(1.0f - tone_decay.Value());
    tone_drum.SetFmAmount(fm_amount.Value() * 0.1);
    tone_drum.SetFmLength(1.0f - fm_decay.Value());

    noise_drum.SetFreq(daisysp::fmap(noise_freq.Value(), 5.0f, 4000.0f));
    noise_drum.SetLength(1.0f - noise_decay.Value());

    tone_drum.SetAmp(tone_amp.Value());
    noise_drum.SetAmp(noise_amp.Value());

    for (grids::PatternGenerator& patgen : patgens) {
        patgen.x = x_ctrl.Value();
        patgen.y = y_ctrl.Value();
        patgen.chaos = chaos.Value();
    }
    patgens[0].fill = tone_fill.Value();
    patgens[1].fill = noise_fill.Value();

    for (size_t i = 0; i < size; i++)
    {
        if (hw.switches[1].Read() == daisy::Switch3::POS_LEFT) {
            gates[0] = hw.som.gate_in_1.State();
            gates[1] = hw.som.gate_in_2.State();
        } else {
            clock.Process();
            if (clock.RisingEdge()) {
                Process();
                hw.leds[2].Set(1.0f);
            }
            if (clock.FallingEdge()) {
                hw.leds[2].Set(0.0f);
                gates[0] = false;
                gates[1] = false;
            }
        }

        hw.som.gate_out_1.Write(gates[0]);
        hw.som.gate_out_2.Write(gates[1]);
        
        OUT_L[i] = tone_drum.Process(gates[0]) + IN_L[i];
        OUT_R[i] = noise_drum.Process(gates[1]) + IN_R[i];

        hw.leds[0].Set(tone_drum.GetCurrAmp());
        hw.leds[1].Set(noise_drum.GetCurrAmp());
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

    knobs_vco.Init(hw, 1, (1 << daisy::Switch3::POS_LEFT) | (1 << daisy::Switch3::POS_CENTER));
    knobs_seq.Init(hw, 1, (1 << daisy::Switch3::POS_RIGHT));

    tone_decay.Init(&knobs_vco.Get(0), nullptr);
    tone_freq.Init(&knobs_vco.Get(1), nullptr);
    fm_amount.Init(&knobs_vco.Get(2), nullptr);
    fm_decay.Init(&knobs_vco.Get(3), nullptr);

    noise_decay.Init(&knobs_vco.Get(4), nullptr);
    noise_freq.Init(&knobs_vco.Get(5), nullptr);
    tone_amp.Init(&knobs_vco.Get(6), nullptr);
    noise_amp.Init(&knobs_vco.Get(7), nullptr);

    x_ctrl.Init(&knobs_seq.Get(0), nullptr);
    y_ctrl.Init(&knobs_seq.Get(1), nullptr);
    chaos.Init(&knobs_seq.Get(2), nullptr);

    tone_fill.Init(&knobs_seq.Get(4), nullptr);
    noise_fill.Init(&knobs_seq.Get(5), nullptr);

    patgens[0].SetInstrument(0);
    patgens[1].SetInstrument(1);

    hw.som.StartLog(false);

    clock.Init(&hw.som.gate_in_1, &knobs_seq.Get(3), hw.som.AudioSampleRate());

    tone_drum.Init(hw.som.AudioSampleRate());
    noise_drum.Init(hw.som.AudioSampleRate());
    
    hw.StartAudio(AudioCallback);

    while(1) {
        daisy::System::Delay(100);
    }
}