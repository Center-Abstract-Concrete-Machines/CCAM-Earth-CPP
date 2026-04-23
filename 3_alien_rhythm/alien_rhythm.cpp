#include "daisysp.h"
#include "Synthesis/oscillator.h"
#include "Filters/svf.h"
#include "Control/adenv.h"

#include <ccam/hw/estuary.h>
#include <ccam/utils/gubbins.h>
#include <cmath>

ccam::hw::Estuary hw;

// --- 3 oscillators for chord voices ---
daisysp::Oscillator osc[3];

// --- Amplitude envelopes (AD) ---
daisysp::AdEnv amp_env[3];

// --- Filter ---
daisysp::Svf filt;

// --- Filter envelope ---
daisysp::AdEnv filt_env;

// --- Klee shift registers (16-bit LFSR) ---
uint16_t klee[3] = {0xACE1, 0xBEEF, 0xCAFE};
int8_t voice_note_idx[3] = {0, 4, 7}; // C, G, Bb in Dorian

// --- Dorian scale from middle C (MIDI notes) ---
// C D Eb F G A Bb | C D Eb F G A Bb C
static const uint8_t dorian[] = {
    60, 62, 63, 65, 67, 69, 70,
    72, 74, 75, 77, 79, 81, 82, 84
};
static constexpr uint8_t DORIAN_LEN = 15;

// --- Step counter ---
uint8_t step = 0;
bool step_active = false;

// --- State ---
float sample_rate_g = 48000.0f;
bool bass_state = false;

// --- Gate output pulse counters ---
int gate_out_1_counter = 0;
int gate_out_2_counter = 0;
static constexpr int GATE_PULSE_LEN = 240; // ~5ms at 48kHz

// --- MIDI note to frequency ---
static float mtof(uint8_t midi) {
    return 440.0f * powf(2.0f, (static_cast<float>(midi) - 69.0f) / 12.0f);
}

// --- Klee step: advance shift register and update voice pitch ---
static void klee_advance(uint8_t v, float chance, float jump_range) {
    // Gate entire note change on chance — at 0, notes freeze
    if (randf() >= chance) return;

    // LFSR feedback (taps for near-maximal-length 16-bit)
    uint16_t bit = ((klee[v] >> 0) ^ (klee[v] >> 1) ^
                    (klee[v] >> 3) ^ (klee[v] >> 12)) & 1;
    klee[v] = (klee[v] >> 1) | (bit << 15);

    // Mutation: flip a random bit for extra variation
    uint8_t flip_pos = static_cast<uint8_t>(randf() * 15.99f);
    klee[v] ^= (1u << flip_pos);

    // Prevent stuck-at-zero
    if (klee[v] == 0) klee[v] = 0xACE1;

    // Determine jump from lower bits of shift register
    int max_steps = 1 + static_cast<int>(jump_range * 13.0f); // 1..14 scale degrees
    int raw_jump = static_cast<int>(klee[v] & 0x1F) % (max_steps * 2 + 1) - max_steps;

    int new_idx = voice_note_idx[v] + raw_jump;
    if (new_idx < 0) new_idx = 0;
    if (new_idx >= DORIAN_LEN) new_idx = DORIAN_LEN - 1;
    voice_note_idx[v] = static_cast<int8_t>(new_idx);
}

static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out,
            size_t size) {
    hw.ProcessAllControls();

    // --- Read knobs ---
    // Dial 1 = attack, Dial 2 = decay, Dial 3 = filter Q, Dial 4 = filter freq
    // Dial 5 = sync/hocket, Dial 6 = voice chance, Dial 7 = jump range, Dial 8 = filter env atk
    float attack       = hw.Knob(0);
    float decay        = hw.Knob(1);
    float filt_q       = hw.Knob(2);
    float filt_fq      = hw.Knob(3);
    float sync_hocket  = hw.Knob(4);
    float voice_chance = hw.Knob(5);
    float jump_range   = hw.Knob(6);
    float filt_env_atk = hw.Knob(7);

    // --- Map parameters ---
    float atk_time = daisysp::fmap(attack, 0.001f, 0.5f);
    float dec_time = daisysp::fmap(decay, 0.01f, 2.0f);
    float filt_atk_time = daisysp::fmap(filt_env_atk, 0.001f, 0.3f);

    // --- Set envelope times ---
    for (int v = 0; v < 3; v++) {
        amp_env[v].SetTime(daisysp::ADENV_SEG_ATTACK, atk_time);
        amp_env[v].SetTime(daisysp::ADENV_SEG_DECAY, dec_time);
    }
    filt_env.SetTime(daisysp::ADENV_SEG_ATTACK, filt_atk_time);
    filt_env.SetTime(daisysp::ADENV_SEG_DECAY, dec_time * 0.5f);

    // --- Switch 1: Waveform ---
    int wave_mode = hw.switches[0].Read();
    uint8_t waveform;
    if (wave_mode == 1) {
        waveform = daisysp::Oscillator::WAVE_SIN;
    } else if (wave_mode == 2) {
        waveform = daisysp::Oscillator::WAVE_POLYBLEP_SAW;
    } else {
        waveform = daisysp::Oscillator::WAVE_POLYBLEP_TRI;
    }
    for (int v = 0; v < 3; v++) {
        osc[v].SetWaveform(waveform);
    }

    // --- Switch 2: Filter mode ---
    int filt_mode = hw.switches[1].Read();

    // --- CV inputs ---
    float cv_filt_fq = hw.cvins[0]->Value(); // bipolar -1..+1

    // --- Per-sample loop ---
    for (size_t i = 0; i < size; i++) {
        bool clock_tick = hw.GateIn1().Trig();
        bass_state = hw.GateIn2().State();

        if (clock_tick) {
            step = (step + 1) % 16;

            // Event logic: sync/hocket
            bool trigger = false;
            if (sync_hocket < 0.33f) {
                trigger = bass_state;           // Sync: fire with bass
            } else if (sync_hocket > 0.66f) {
                trigger = !bass_state;          // Hocket: fire without bass
            } else {
                trigger = true;                 // Middle: always fire
            }

            step_active = trigger;

            if (trigger) {
                // Advance Klee registers and update pitches
                for (int v = 0; v < 3; v++) {
                    klee_advance(v, voice_chance, jump_range);
                    osc[v].SetFreq(mtof(dorian[voice_note_idx[v]]));
                    amp_env[v].Trigger();
                }
                filt_env.Trigger();
                gate_out_1_counter = GATE_PULSE_LEN;
            } else {
                gate_out_2_counter = GATE_PULSE_LEN;
            }

            // LEDs: 16 steps across 8 LEDs, bright if active, dim if inactive
            for (uint8_t j = 0; j < 8; j++) {
                hw.leds[j].Set(0.0f);
            }
            hw.leds[step / 2].Set(trigger ? 1.0f : 0.15f);
        }

        // --- Process synthesis ---
        float mix = 0.0f;
        for (int v = 0; v < 3; v++) {
            float env_val = amp_env[v].Process();
            mix += osc[v].Process() * env_val;
        }
        mix *= 0.33f;

        // --- Filter ---
        float fenv = filt_env.Process();
        float fq_in = daisysp::fclamp(filt_fq + cv_filt_fq * 0.5f, 0.0f, 1.0f);
        float base_fq = daisysp::fmap(fq_in, 80.0f, 12000.0f, daisysp::Mapping::LOG);
        float env_fq = base_fq + fenv * 5000.0f;
        if (env_fq > 18000.0f) env_fq = 18000.0f;

        filt.SetFreq(env_fq);
        filt.SetRes(daisysp::fmap(filt_q, 0.1f, 0.95f));
        filt.Process(mix);

        float filtered;
        if (filt_mode == 1) {
            filtered = filt.Low();
        } else if (filt_mode == 2) {
            filtered = filt.High();
        } else {
            filtered = filt.Band();
        }

        // --- Output ---
        OUT_L[i] = filtered + IN_L[i];
        OUT_R[i] = filtered + IN_R[i];

        // --- Gate outputs ---
        if (gate_out_1_counter > 0) gate_out_1_counter--;
        if (gate_out_2_counter > 0) gate_out_2_counter--;
        hw.GateOut1().Write(gate_out_1_counter > 0);
        hw.GateOut2().Write(hw.GateIn1().State());

        // --- CV outputs: top and bottom voice pitch (1V/oct, 0V = middle C) ---
        int top_idx = 0, bot_idx = 0;
        for (int v = 1; v < 3; v++) {
            if (voice_note_idx[v] > voice_note_idx[top_idx]) top_idx = v;
            if (voice_note_idx[v] < voice_note_idx[bot_idx]) bot_idx = v;
        }
        // True 1V/oct: 0V = middle C (MIDI 60), +1V = octave up
        float cv_top = (static_cast<float>(dorian[voice_note_idx[top_idx]]) - 60.0f) / 12.0f;
        float cv_bot = (static_cast<float>(dorian[voice_note_idx[bot_idx]]) - 60.0f) / 12.0f;
        hw.WriteCvOut(daisy::patch_sm::CV_OUT_1, cv_top);
        hw.WriteCvOut(daisy::patch_sm::CV_OUT_2, cv_bot);
    }

    hw.PostProcess();
}

int main(void) {
    hw.Init();

    for (size_t i = 0; i < 10; i++) {
        daisy::System::Delay(1);
        hw.ProcessAllControls();
    }

    sample_rate_g = hw.AudioSampleRate();

    // --- Init oscillators ---
    for (int v = 0; v < 3; v++) {
        osc[v].Init(sample_rate_g);
        osc[v].SetWaveform(daisysp::Oscillator::WAVE_POLYBLEP_TRI);
        osc[v].SetAmp(0.8f);
        osc[v].SetFreq(mtof(dorian[voice_note_idx[v]]));
    }

    // --- Init amplitude envelopes ---
    for (int v = 0; v < 3; v++) {
        amp_env[v].Init(sample_rate_g);
        amp_env[v].SetTime(daisysp::ADENV_SEG_ATTACK, 0.01f);
        amp_env[v].SetTime(daisysp::ADENV_SEG_DECAY, 0.3f);
        amp_env[v].SetMax(1.0f);
        amp_env[v].SetMin(0.0f);
    }

    // --- Init filter envelope ---
    filt_env.Init(sample_rate_g);
    filt_env.SetTime(daisysp::ADENV_SEG_ATTACK, 0.005f);
    filt_env.SetTime(daisysp::ADENV_SEG_DECAY, 0.15f);
    filt_env.SetMax(1.0f);
    filt_env.SetMin(0.0f);

    // --- Init filter ---
    filt.Init(sample_rate_g);
    filt.SetFreq(1000.0f);
    filt.SetRes(0.5f);

    // --- Start audio ---
    hw.StartAudio(AudioCallback);

    while (1) {
        daisy::System::Delay(100);
    }
}
