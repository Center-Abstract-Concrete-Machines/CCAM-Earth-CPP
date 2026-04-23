#include "daisysp.h"
#include <cmath>
#include "ccam/hw/estuary.h"
#include "ccam/utils/gubbins.h"

using namespace daisy;
using namespace daisysp;

static ccam::hw::Estuary hw;

// ---------- Turing Machine ----------
static uint16_t turing_reg = 0xACE1;

// D Dorian bass scale C2–D4 (15 notes)
static constexpr int SCALE_LEN = 15;
static constexpr uint8_t scale[SCALE_LEN] = {
    38, 39, 41, 43, 45, 46, 48,
    50, 51, 53, 55, 57, 58, 60, 62
};

static void turing_step(float loop_prob, bool half_length) {
    uint8_t tap = half_length ? 7 : 15;
    uint8_t feedback_bit = (turing_reg >> tap) & 1;

    float flip_chance = 1.0f - loop_prob;
    if (randf() < flip_chance) {
        feedback_bit ^= 1;
    }

    turing_reg = (turing_reg >> 1) | (static_cast<uint16_t>(feedback_bit) << 15);

    if (half_length) {
        turing_reg = (turing_reg & ~(1u << 7)) |
                     (static_cast<uint16_t>(feedback_bit) << 7);
    }
}

static uint8_t reg_to_note(uint8_t byte_val) {
    int idx = (byte_val * SCALE_LEN) >> 8;
    if (idx >= SCALE_LEN) idx = SCALE_LEN - 1;
    return scale[idx];
}

// ---------- Synthesis ----------
static HarmonicOscillator<16> harm_osc;
static Oscillator sub_osc;
static AdEnv amp_env;
static Svf filt;

// Simple one-pole portamento
static float glide_freq = 65.41f;
static float glide_coeff = 0.99f;

// ---------- State ----------
static float target_freq = 65.41f;

static void AudioCallback(daisy::AudioHandle::InputBuffer  in,
                          daisy::AudioHandle::OutputBuffer out,
                          size_t                           size) {
    hw.ProcessAllControls();

    // --- Knobs ---
    // Top row: tone shaping
    float attack    = hw.Knob(0);  // 1: attack
    float decay     = hw.Knob(1);  // 2: decay
    float harmonics = hw.Knob(2);  // 3: harmonic richness
    float filt_knob = hw.Knob(3);  // 4: filter cutoff base

    // Bottom row
    float loop_prob = hw.Knob(4);  // 5: looping
    float glide_amt = hw.Knob(5);  // 6: glide / portamento
    float oct_knob  = hw.Knob(6);  // 7: octave shift
    float sub_mix   = hw.Knob(7);  // 8: sub-bass mix

    // --- Switch 1: step length ---
    bool half_length = (hw.switches[0].Read() == 2);  // DOWN = 8-step

    // --- Switch 2: filter type (LP / HP / BP) ---
    int filt_mode = hw.switches[1].Read();  // 1=LP, 2=HP, 0=BP

    // --- CV / Gate ---
    float filter_cv = hw.cvins[0]->Value();  // bipolar -1..+1
    bool  trig      = hw.GateIn1().Trig();   // clock
    bool  bass_high = hw.GateIn2().State();  // bass accent

    // --- Octave shift: CCW = -2 oct, noon = 0, CW = +2 oct ---
    int oct_shift = static_cast<int>(oct_knob * 4.0f + 0.5f) - 2;  // -2..+2

    if (trig) {
        turing_step(loop_prob, half_length);

        uint8_t lo_byte = turing_reg & 0xFF;
        uint8_t hi_byte = (turing_reg >> 8) & 0xFF;
        uint8_t play_note = reg_to_note(lo_byte);
        uint8_t alt_note  = reg_to_note(hi_byte);

        // Apply octave shift
        int shifted = static_cast<int>(play_note) + oct_shift * 12;
        if (shifted < 12) shifted = 12;
        if (shifted > 108) shifted = 108;

        target_freq = mtof(static_cast<float>(shifted));
        amp_env.Trigger();

        // Gate outs
        hw.GateOut1().Write(bass_high);
        hw.GateOut2().Write(!bass_high);

        // CV outs: V/oct (0V = MIDI 60)
        float cv_top = (static_cast<float>(alt_note)  - 60.0f) / 12.0f;
        float cv_bot = (static_cast<float>(shifted)   - 60.0f) / 12.0f;
        hw.WriteCvOut(patch_sm::CV_OUT_1, cv_top);
        hw.WriteCvOut(patch_sm::CV_OUT_2, cv_bot);
    }

    // --- LEDs: 8 bits of register ---
    for (int i = 0; i < 8; i++) {
        hw.leds[i].Set((turing_reg >> i) & 1 ? 1.0f : 0.0f);
    }
    hw.PostProcess();

    // --- Envelope params ---
    amp_env.SetTime(ADENV_SEG_ATTACK, 0.001f + attack * 0.3f);
    amp_env.SetTime(ADENV_SEG_DECAY,  0.05f  + decay  * 2.0f);

    // --- Glide: 0 = instant, 1 = slow portamento ---
    // Compute one-pole coefficient from glide_amt (higher = slower)
    float glide_time = 0.001f + glide_amt * 0.5f;  // 1ms to 500ms
    glide_coeff = expf(-1.0f / (glide_time * hw.AudioSampleRate()));

    // --- Harmonic amplitudes ---
    float amplitudes[16];
    amplitudes[0] = 1.0f;
    for (int i = 1; i < 16; i++) {
        amplitudes[i] = harmonics / static_cast<float>(i + 1);
    }
    harm_osc.SetAmplitudes(amplitudes);

    // --- Filter ---
    float filt_base = 100.0f + filt_knob * 3000.0f;
    float filt_mod  = filter_cv * 5.0f;
    float filt_freq = filt_base * powf(2.0f, filt_mod);
    if (filt_freq > 18000.0f) filt_freq = 18000.0f;
    if (filt_freq < 20.0f)    filt_freq = 20.0f;
    filt.SetFreq(filt_freq);
    filt.SetRes(0.3f);

    // --- Audio ---
    for (size_t i = 0; i < size; i++) {
        glide_freq = glide_freq * glide_coeff + target_freq * (1.0f - glide_coeff);
        harm_osc.SetFreq(glide_freq);
        sub_osc.SetFreq(glide_freq * 0.5f);  // sub = 1 octave below

        float env = amp_env.Process();
        float sig = harm_osc.Process() * env;
        float sub = sub_osc.Process() * env * sub_mix;

        filt.Process(sig + sub);

        float filtered;
        if (filt_mode == 1) {
            filtered = filt.Low();
        } else if (filt_mode == 2) {
            filtered = filt.High();
        } else {
            filtered = filt.Band();
        }

        OUT_L[i] = filtered + IN_L[i];
        OUT_R[i] = filtered + IN_R[i];
    }

    // Clear gate outs after block
    hw.GateOut1().Write(false);
    hw.GateOut2().Write(false);
}

int main() {
    hw.Init();

    float sr = hw.AudioSampleRate();

    harm_osc.Init(sr);
    harm_osc.SetFreq(65.41f);  // C2
    harm_osc.SetFirstHarmIdx(1);

    sub_osc.Init(sr);
    sub_osc.SetFreq(32.7f);
    sub_osc.SetWaveform(Oscillator::WAVE_SIN);
    sub_osc.SetAmp(1.0f);

    amp_env.Init(sr);
    amp_env.SetMin(0.0f);
    amp_env.SetMax(1.0f);
    amp_env.SetCurve(0.0f);
    amp_env.SetTime(ADENV_SEG_ATTACK, 0.01f);
    amp_env.SetTime(ADENV_SEG_DECAY,  0.5f);

    filt.Init(sr);
    filt.SetFreq(1000.0f);
    filt.SetRes(0.3f);

    hw.StartAudio(AudioCallback);

    while (1) {}
}
