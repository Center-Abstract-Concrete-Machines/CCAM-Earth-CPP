#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY
#include <ccam/hw/estuary.h>
#include "daisysp.h"
#include <cstdlib>

using namespace daisy;
using namespace daisysp;

static constexpr size_t MAX_BUF = 192001; // ~4s at 48kHz (whole note @ 60 BPM)

float DSY_SDRAM_BSS delay_buf_l[MAX_BUF];
float DSY_SDRAM_BSS delay_buf_r[MAX_BUF];

static ccam::hw::Estuary hw;

enum GateState { GATE_OFF, GATE_FWD, GATE_BWD };
static GateState gate_state         = GATE_OFF;
static size_t    write_ptr          = 0;
static size_t    fwd_read_ptr       = 0;
static size_t    bwd_read_ptr       = 0;
static size_t    actual_delay_samps = 48000;
static float     gate_amplitude     = 0.0f;
static float     sample_rate        = 48000.0f;

// Rhythmic subdivisions as fractions of a quarter note
static const float delay_ratios[10] = {
    1.0f/6.0f,  // 16th triplet
    1.0f/4.0f,  // 16th
    1.0f/3.0f,  // 8th triplet
    1.0f/2.0f,  // 8th
    2.0f/3.0f,  // quarter triplet
    1.0f,       // quarter
    4.0f/3.0f,  // half triplet
    2.0f,       // half
    8.0f/3.0f,  // whole triplet
    4.0f        // whole
};

inline float interpf(float a, float b, float t) { return a * (1.0f - t) + b * t; }

inline size_t varied_delay(size_t base, float rand_pct) {
    float jitter = ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * rand_pct * (float)base;
    return (size_t)fclamp((float)base + jitter, 1.0f, (float)(MAX_BUF - 1));
}

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size) {
    hw.ProcessAllControls();

    float bpm       = fmap(hw.Knob(6), 60.0f, 180.0f);
    float quarter_s = 60.0f / bpm;
    int   step      = (int)(hw.Knob(0) * 9.99f);
    float delay_sec = quarter_s * delay_ratios[step];
    size_t base_delay = (size_t)fclamp(delay_sec * sample_rate, 1.0f, (float)(MAX_BUF - 1));

    float rand_pct   = hw.Knob(1);
    float feedback   = hw.Knob(2) * 0.9f;
    float delay_vol  = hw.Knob(3);
    float dry_wet    = hw.Knob(4);
    float smooth_ms  = fmap(hw.Knob(5), 1.0f, 200.0f);
    float smooth_inc = 1.0f / (smooth_ms * 0.001f * sample_rate);

    bool insert_mode     = (hw.switches[0].Read() == Switch3::POS_LEFT);
    bool ducking_enabled = (hw.switches[1].Read() == Switch3::POS_RIGHT);

    bool g1_trig  = hw.GateIn1().Trig();
    bool g1_state = hw.GateIn1().State();
    bool g2_trig  = hw.GateIn2().Trig();
    bool g2_state = hw.GateIn2().State();

    if (g1_trig) {
        actual_delay_samps = varied_delay(base_delay, rand_pct);
        fwd_read_ptr = (write_ptr + MAX_BUF - actual_delay_samps) % MAX_BUF;
    }
    if (g2_trig) {
        actual_delay_samps = varied_delay(base_delay, rand_pct);
        bwd_read_ptr = (write_ptr + MAX_BUF - 1) % MAX_BUF;
    }

    if      (g1_state) gate_state = GATE_FWD;
    else if (g2_state) gate_state = GATE_BWD;
    else               gate_state = GATE_OFF;

    for (size_t i = 0; i < size; i++) {
        float in_l = IN_L[i];
        float in_r = IN_R[i];

        float raw_l = 0.0f, raw_r = 0.0f;
        if (gate_state == GATE_FWD) {
            raw_l = delay_buf_l[fwd_read_ptr];
            raw_r = delay_buf_r[fwd_read_ptr];
            fwd_read_ptr = (fwd_read_ptr + 1) % MAX_BUF;
        } else if (gate_state == GATE_BWD) {
            raw_l = delay_buf_l[bwd_read_ptr];
            raw_r = delay_buf_r[bwd_read_ptr];
            bwd_read_ptr = (bwd_read_ptr + MAX_BUF - 1) % MAX_BUF;
        }

        delay_buf_l[write_ptr] = in_l + feedback * raw_l;
        delay_buf_r[write_ptr] = in_r + feedback * raw_r;
        write_ptr = (write_ptr + 1) % MAX_BUF;

        float target = (gate_state != GATE_OFF) ? 1.0f : 0.0f;
        float step_dir = (target > gate_amplitude) ? 1.0f : -1.0f;
        gate_amplitude = fclamp(gate_amplitude + smooth_inc * step_dir, 0.0f, 1.0f);

        float dry_gain = insert_mode ? 1.0f : gate_amplitude;
        float duck     = ducking_enabled ? (1.0f - gate_amplitude) : 1.0f;

        float delay_out_l = raw_l * gate_amplitude * delay_vol;
        float delay_out_r = raw_r * gate_amplitude * delay_vol;
        float dry_l = in_l * dry_gain * duck;
        float dry_r = in_r * dry_gain * duck;

        OUT_L[i] = interpf(dry_l, delay_out_l, dry_wet);
        OUT_R[i] = interpf(dry_r, delay_out_r, dry_wet);
    }

    hw.leds[0].Set(gate_state == GATE_FWD ? 1.0f : 0.0f);
    hw.leds[1].Set(gate_state == GATE_BWD ? 1.0f : 0.0f);
    hw.leds[2].Set(gate_amplitude);
    hw.leds[3].Set(insert_mode ? 1.0f : 0.25f);
    hw.leds[4].Set(ducking_enabled ? 1.0f : 0.0f);
    hw.leds[5].Set((float)step / 9.0f);
    hw.leds[6].Set(feedback / 0.9f);
    hw.leds[7].Set(delay_vol);

    hw.PostProcess();
}

int main(void) {
    hw.Init();
    sample_rate = hw.AudioSampleRate();

    for (size_t i = 0; i < MAX_BUF; i++) {
        delay_buf_l[i] = 0.0f;
        delay_buf_r[i] = 0.0f;
    }

    hw.StartAudio(AudioCallback);
    while (1) { System::Delay(100); }
}
