// note still funny business with knob 3 and 4 cutoff scaling! 

#include <ccam/hw/estuary.h>
#include <ccam/voice/tonedrum.h>
#include <ccam/voice/noisedrum.h>
#include <ccam/utils/gubbins.h>
#include <ccam/utils/perlin.h>
#include <ccam/seq/gridseq.h>

#include "daisysp.h"
#include <cmath>
#include "Drums/analogbassdrum.h"
#include "Drums/synthbassdrum.h"
#include "Drums/analogsnaredrum.h"
#include "Drums/synthsnaredrum.h"
#include "Drums/hihat.h"
#include "Filters/svf.h"

ccam::hw::Estuary hw;

// --- Kick voices (3 models) ---
daisysp::AnalogBassDrum    kick_808;
daisysp::SyntheticBassDrum kick_909;
ToneDrum                   kick_basic;

// --- Snare voices (3 models) ---
daisysp::AnalogSnareDrum    snare_808;
daisysp::SyntheticSnareDrum snare_909;
NoiseDrum                   snare_basic;

// --- HiHat ---
daisysp::HiHat<daisysp::SquareNoise> hihat;

// --- Euclidean sequencer for kick ---
grids::EuclidianGenerator euclid;

// --- Sidechain HPFs ---
daisysp::Svf sidechain_hpf_sn;
daisysp::Svf sidechain_hpf_hh;

// --- Clock state ---
float    clock_phase      = 0.0f;
float    logistic_x       = 0.5f;
uint8_t  step             = 0;
float    sidechain_freq   = 20.0f;
float    sample_rate_g    = 48000.0f;

// --- Gate output pulse counters ---
int gate_out_1_counter = 0;
int gate_out_2_counter = 0;
static constexpr int GATE_PULSE_LEN = 240; // ~5ms at 48kHz

// --- External clock detection ---
bool   ext_clock_active   = false;
size_t last_ext_tick      = 0;
float  ext_clock_freq     = 2.0f;
static constexpr float EXT_CLOCK_TIMEOUT = 2.0f;

// --- CV output values ---
float cv_out_1 = 0.0f;
float cv_out_2 = 0.0f;

// --- Trigger flags (held for one sample on clock tick) ---
bool kick_trig_flag  = false;
bool snare_trig_flag = false;
bool hh_trig_flag    = false;

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out,
            size_t size) {
    hw.ProcessAllControls();

    // --- Read knobs ---
    // Dial 1 = BPM, Dial 2 = kick fill, Dial 3 = HH density, Dial 4 = SN density
    // Dial 5 = chaos, Dial 6 = kick length, Dial 7 = HH Y offset, Dial 8 = SN Y offset
    float bpm_val     = hw.Knob(0);
    float bd_fill     = hw.Knob(1);
    float hh_density  = hw.Knob(2);
    float sn_density  = hw.Knob(3);
    // Small dead zone as extra insurance for cutoff
    hh_density = hh_density < 0.02f ? 0.0f : hh_density;
    sn_density = sn_density < 0.02f ? 0.0f : sn_density;
    float chaos_val   = hw.Knob(4);
    float bd_len_raw  = hw.Knob(5);
    float hh_y_offset = hw.Knob(6) * 10.0f;
    float sn_y_offset = hw.Knob(7) * 10.0f + 50.0f;

    // --- Map BPM and chaos ---
    float bpm       = daisysp::fmap(bpm_val, 20.0f, 300.0f);
    float base_freq = bpm / 60.0f * 4.0f; // quarter-note BPM -> 16th-note step freq
    float chaos_r   = daisysp::fmap(chaos_val, 3.5f, 3.99f);

    // --- Euclidean parameters ---
    uint8_t bd_length = static_cast<uint8_t>(daisysp::fclamp(bd_len_raw * 31.0f + 1.0f, 1.0f, 32.0f));
    euclid.fill = bd_fill;
    euclid.SetLength(bd_length);

    // --- CV inputs ---
    float hh_pan     = daisysp::fclamp(hw.cvins[0]->Value(), -1.0f, 1.0f);
    float snare_freq = 200.0f + hw.cvins[1]->Value() * 100.0f;
    // V/oct tracking: cvins bipolar maps -5V..+5V to -1..+1
    // Recover voltage: val * 5.0. Base kick at C2 (65.41Hz), exponential tracking.
    // Modulo 1 octave so pitch wraps and stays in kick register (C2-C3)
    float kick_cv_volts = hw.cvins[2]->Value() * 5.0f;
    kick_cv_volts = fmodf(kick_cv_volts, 1.0f);
    if (kick_cv_volts < 0.0f) kick_cv_volts += 1.0f;
    float kick_freq  = 65.41f * powf(2.0f, kick_cv_volts);

    // --- Set drum frequencies on ALL voices (switch may change mid-sound) ---
    int kick_mode  = hw.switches[0].Read();
    int snare_mode = hw.switches[1].Read();

    kick_808.SetFreq(kick_freq);
    kick_909.SetFreq(kick_freq);
    kick_basic.SetFreq(kick_freq);

    snare_808.SetFreq(snare_freq);
    snare_909.SetFreq(snare_freq);
    snare_basic.SetFreq(snare_freq);

    // --- Per-sample loop ---
    for (size_t i = 0; i < size; i++) {
        bool clock_tick = false;

        // --- External clock detection ---
        size_t curr_sys_tick = daisy::System::GetTick();
        float sys_freq = static_cast<float>(daisy::System::GetTickFreq());

        if (hw.GateIn1().Trig()) {
            if (last_ext_tick > 0) {
                ext_clock_freq = sys_freq / static_cast<float>(curr_sys_tick - last_ext_tick);
            }
            last_ext_tick = curr_sys_tick;
            ext_clock_active = true;
            clock_tick = true;
        }

        float time_since_last = static_cast<float>(curr_sys_tick - last_ext_tick) / sys_freq;
        if (time_since_last >= EXT_CLOCK_TIMEOUT) {
            ext_clock_active = false;
        }

        // --- Internal clock with logistic map modulation ---
        if (!ext_clock_active) {
            float swing_factor = 1.0f + (logistic_x - 0.5f) * chaos_val;
            float phase_inc = base_freq * swing_factor / sample_rate_g;
            clock_phase += phase_inc;
            if (clock_phase >= 1.0f) {
                clock_phase -= 1.0f;
                clock_tick = true;
            }
        }

        // --- Step processing on clock tick ---
        kick_trig_flag  = false;
        snare_trig_flag = false;
        hh_trig_flag    = false;

        if (clock_tick) {
            step = (step + 1) % 16;

            // Euclidean kick
            euclid.Tick();
            kick_trig_flag = euclid.Triggered();

            // Perlin noise for hihat and snare (shared X axis, separate Y offsets)
            static constexpr float PERLIN_STEP_SCALE = 0.5f;
            float shared_x = step * PERLIN_STEP_SCALE;

            float hh_noise = (Perlin2D(shared_x, hh_y_offset) + 1.0f) * 0.5f;
            float sn_noise = (Perlin2D(shared_x, sn_y_offset) + 1.0f) * 0.5f;

            hh_trig_flag    = (hh_density > 0.0f) && (hh_noise > (1.0f - hh_density));
            snare_trig_flag = (sn_density > 0.0f) && (sn_noise > (1.0f - sn_density));

            // Update logistic map for next step's swing
            logistic_x = chaos_r * logistic_x * (1.0f - logistic_x);
            logistic_x = daisysp::fclamp(logistic_x, 0.001f, 0.999f);

            // Gate output pulses
            gate_out_1_counter = GATE_PULSE_LEN;
            if (kick_trig_flag) {
                gate_out_2_counter = GATE_PULSE_LEN;
            }

            // CV outputs
            cv_out_1 = (hh_noise) * 5.0f;
            cv_out_2 = logistic_x * 5.0f;

            // LEDs 0-7: step sequencer position (16 steps across 8 LEDs)
            for (uint8_t j = 0; j < 8; j++) {
                hw.leds[j].Set(0.0f);
            }
            hw.leds[step / 2].Set(1.0f);

            // Sidechain: boost HPF cutoff on kick
            if (kick_trig_flag) {
                sidechain_freq = 500.0f;
            }
        }

        // --- Open hihat from gate_in_2 ---
        bool open_hh_trig = hw.GateIn2().Trig();
        if (open_hh_trig) {
            hihat.SetDecay(0.7f);
        }

        // Closed hihat from sequencer overrides open
        if (hh_trig_flag) {
            hihat.SetDecay(0.15f);
        }

        bool hh_trigger = hh_trig_flag || open_hh_trig;

        // --- Process ALL drum voices (maintain state), select by switch ---
        float k808  = kick_808.Process(kick_trig_flag);
        float k909  = kick_909.Process(kick_trig_flag);
        float kbas  = kick_basic.Process(kick_trig_flag);

        float kick_out;
        if (kick_mode == 1) {
            kick_out = k808;
        } else if (kick_mode == 2) {
            kick_out = kbas;
        } else {
            kick_out = k909;  // CENTER (0) = 909
        }

        float s808  = snare_808.Process(snare_trig_flag);
        float s909  = snare_909.Process(snare_trig_flag);
        float sbas  = snare_basic.Process(snare_trig_flag);

        float snare_out;
        if (snare_mode == 1) {
            snare_out = s808;
        } else if (snare_mode == 2) {
            snare_out = sbas;
        } else {
            snare_out = s909;  // CENTER (0) = 909
        }

        float hh_out = hihat.Process(hh_trigger);

        // Mute when density is zero
        if (sn_density <= 0.0f) snare_out = 0.0f;
        if (hh_density <= 0.0f) hh_out = 0.0f;

        // --- Sidechain HPF ---
        sidechain_freq *= 0.9998f;
        if (sidechain_freq < 20.0f) sidechain_freq = 20.0f;

        sidechain_hpf_sn.SetFreq(sidechain_freq);
        sidechain_hpf_hh.SetFreq(sidechain_freq);

        sidechain_hpf_sn.Process(snare_out);
        sidechain_hpf_hh.Process(hh_out);

        float sc_snare = sidechain_hpf_sn.High();
        float sc_hh    = sidechain_hpf_hh.High();

        // --- Stereo mix with hihat panning ---
        float left_gain  = (1.0f - hh_pan) * 0.5f;
        float right_gain = (1.0f + hh_pan) * 0.5f;

        OUT_L[i] = kick_out * 0.4f + sc_snare * 0.3f + sc_hh * left_gain * 0.6f + IN_L[i];
        OUT_R[i] = kick_out * 0.4f + sc_snare * 0.3f + sc_hh * right_gain * 0.6f + IN_R[i];

        // --- Gate outputs ---
        if (gate_out_1_counter > 0) gate_out_1_counter--;
        if (gate_out_2_counter > 0) gate_out_2_counter--;

        hw.GateOut1().Write(gate_out_1_counter > 0);
        hw.GateOut2().Write(gate_out_2_counter > 0);

        // --- CV outputs ---
        hw.WriteCvOut(daisy::patch_sm::CV_OUT_1, cv_out_1);
        hw.WriteCvOut(daisy::patch_sm::CV_OUT_2, cv_out_2);
    }

    // LEDs 6-7: switch position indicators (always set, not overwritten by step seq)
    // (disabled — all 8 LEDs used for step sequencer)

    hw.PostProcess();
}

int main(void) {
    hw.Init();

    // Warm-up loop to stabilize ADC/knob values
    for (size_t i = 0; i < 10; i++) {
        daisy::System::Delay(1);
        hw.ProcessAllControls();
    }

    sample_rate_g = hw.AudioSampleRate();

    // --- Init kick voices (distinct character per model) ---
    kick_808.Init(sample_rate_g);
    kick_808.SetFreq(45.0f);
    kick_808.SetDecay(0.7f);
    kick_808.SetTone(0.6f);
    kick_808.SetAccent(0.9f);
    kick_808.SetAttackFmAmount(0.3f);
    kick_808.SetSelfFmAmount(0.3f);

    kick_909.Init(sample_rate_g);
    kick_909.SetFreq(55.0f);
    kick_909.SetDecay(0.5f);
    kick_909.SetTone(0.4f);
    kick_909.SetAccent(0.8f);
    kick_909.SetDirtiness(0.6f);
    kick_909.SetFmEnvelopeAmount(0.8f);
    kick_909.SetFmEnvelopeDecay(0.3f);

    kick_basic.Init(sample_rate_g);
    kick_basic.SetFreq(50.0f);
    kick_basic.SetLength(0.4f);
    kick_basic.SetFmAmount(0.5f);
    kick_basic.SetFmLength(0.3f);
    kick_basic.SetAmp(0.9f);

    // --- Init snare voices (distinct character per model) ---
    snare_808.Init(sample_rate_g);
    snare_808.SetFreq(180.0f);
    snare_808.SetDecay(0.5f);
    snare_808.SetTone(0.4f);
    snare_808.SetSnappy(0.5f);
    snare_808.SetAccent(0.9f);

    snare_909.Init(sample_rate_g);
    snare_909.SetFreq(250.0f);
    snare_909.SetDecay(0.3f);
    snare_909.SetFmAmount(0.6f);
    snare_909.SetSnappy(0.8f);
    snare_909.SetAccent(0.8f);

    snare_basic.Init(sample_rate_g);
    snare_basic.SetFreq(300.0f);
    snare_basic.SetLength(0.3f);
    snare_basic.SetAmp(0.9f);

    // --- Init hihat ---
    hihat.Init(sample_rate_g);
    hihat.SetFreq(2000.0f);
    hihat.SetTone(0.5f);
    hihat.SetAccent(1.0f);
    hihat.SetNoisiness(0.9f);
    hihat.SetDecay(0.5f);

    // --- Init sidechain HPFs ---
    sidechain_hpf_sn.Init(sample_rate_g);
    sidechain_hpf_sn.SetRes(0.3f);
    sidechain_hpf_sn.SetFreq(20.0f);

    sidechain_hpf_hh.Init(sample_rate_g);
    sidechain_hpf_hh.SetRes(0.3f);
    sidechain_hpf_hh.SetFreq(20.0f);

    // --- Init Euclidean generator ---
    euclid.SetLength(16);

    // --- Start audio ---
    hw.StartAudio(AudioCallback);

    while (1) {
        daisy::System::Delay(100);
    }
}
