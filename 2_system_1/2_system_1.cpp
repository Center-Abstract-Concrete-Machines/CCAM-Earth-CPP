#include "daisysp.h"

#include <ccam/hw/estuary.h>
#include <ccam/voice/smoothosc.h>
#include <ccam/utils/quantizer.h>
#include <ccam/utils/delayenv.h>
#include <ccam/utils/lockedEstuaryKnobs.h>
#include <ccam/utils/gubbins.h>
#include <ccam/utils/gateclock.h>

ccam::hw::Estuary hw;

GateClock<LockedAnalogControl> clock;

uint8_t step_num = 0;
static uint8_t seq_step = 1;

float midi_start = 10.0f;
float midi_range = 36.0f;
uint8_t scale = 0;
float chance = 1.0;

std::array<SmoothOsc, 2> vcos;
std::array<DelayEnv, 2> envs;
std::array<bool, 2> gates;

LockedEstaury knobs_8x1;
LockedEstaury knobs_2x4;
LockedEstaury knobs_ctrl;

void Step8x1() {
    step_num = (step_num + 1) % 8;
}

void Step4x2() {
    if (step_num >= 3) {
        seq_step = -1;
    } else if (step_num == 0) {
        seq_step = 1;
    }
    step_num = (step_num + seq_step) % 4;
}

float ChannelNoteOffset(uint8_t channel) {
    if (channel == 1) {
        return 0;
    }
    switch (hw.switches[0].Read()) {
        case daisy::Switch3::POS_CENTER:
            return 7.0f;
        case daisy::Switch3::POS_RIGHT:
            return 12.0f;
        default:
            return 0;
    }
}

void WriteStep(uint8_t channel, float value, bool trig) {
    float note = daisysp::fmap(value, midi_start, midi_start + midi_range);
    note += ChannelNoteOffset(channel);
    note = Quantizer::apply(static_cast<Quantizer::Scale>(scale), note);
    float freq = daisysp::mtof(note);
    hw.som.WriteCvOut(channel+1, ftov(freq));
    vcos[channel].SetFreq(freq);
    gates[channel] = trig;
};

void Process() {
    switch(hw.switches[1].Read()) {
        case daisy::Switch3::POS_LEFT:
            Step8x1();
            WriteStep(0, knobs_8x1.Value(step_num), true);
            WriteStep(1, randf(), true);
            hw.leds[step_num].Set(1.0f);
            break;
        case daisy::Switch3::POS_CENTER:
            Step4x2();
            WriteStep(0, knobs_2x4.Value(step_num), true);
            WriteStep(1, knobs_2x4.Value(step_num+4), true);
            hw.leds[step_num].Set(1.0f);
            hw.leds[step_num+4].Set(1.0f);
            break;
        case daisy::Switch3::POS_RIGHT:
            for (size_t i = 0; i < vcos.size(); i++) {
                vcos[i].SetWaveshape(knobs_ctrl.Value(4 + i));
            }
            for (DelayEnv& env : envs) {
                env.SetLength(knobs_ctrl.Value(2));
            }
            chance = hw.knobs[0]->Value();
            midi_start = daisysp::fmap(knobs_ctrl.Value(6), 1.0f, 100.f);
            midi_range = daisysp::fmap(knobs_ctrl.Value(7), 1.0f, 36.0f);
            if (randf() < chance) {
                WriteStep(0, randf(), true);
            }
            if (randf() < chance) {
                WriteStep(1, randf(), true);
            }
            hw.leds[0].Set(gates[0] ? 1.0f : 0.0f);
            hw.leds[1].Set(gates[1] ? 1.0f : 0.0f);
            break;
    }
    hw.som.gate_out_1.Write(gates[0]);
    hw.som.gate_out_2.Write(gates[1]);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    hw.ProcessAllControls();
    knobs_8x1.Process();
    knobs_2x4.Process();
    knobs_ctrl.Process();

    for (uint8_t i = 0; i < hw.leds.size(); i++) {
        hw.leds[i].Set(0.0);
    }

    scale = knobs_ctrl.Value(1) * 8.0f;
    if (hw.switches[1].Read() == daisy::Switch3::POS_RIGHT) {
        hw.leds[scale].Set(1.0f);
    }

    for (size_t i = 0; i < size; i++)
    {
        clock.Process();
        if (clock.RisingEdge()) {
            Process();
        }
        if (clock.FallingEdge()) {
            gates[0] = false;
            gates[1] = false;
            hw.som.gate_out_1.Write(false);
            hw.som.gate_out_2.Write(false);
        }

        OUT_L[i] = vcos[0].Process() * envs[0].Process(gates[0]);
        OUT_R[i] = vcos[1].Process() * envs[1].Process(gates[1]);
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

    knobs_8x1.Init(hw, 1, (1 << daisy::Switch3::POS_LEFT));
    knobs_2x4.Init(hw, 1, (1 << daisy::Switch3::POS_CENTER));
    knobs_ctrl.Init(hw, 1, (1 << daisy::Switch3::POS_RIGHT));
    
    hw.som.StartLog(false);
    clock.Init(&hw.som.gate_in_1, &knobs_ctrl.Get(3), hw.som.AudioSampleRate());

    for (SmoothOsc& vco : vcos) {
        vco.Init(hw.som.AudioSampleRate());
    }

    for (DelayEnv& env : envs) {
        env.Init(hw.som.AudioSampleRate());
    }

    hw.StartAudio(AudioCallback);

    while(1) {
        daisy::System::Delay(100);
    }
}