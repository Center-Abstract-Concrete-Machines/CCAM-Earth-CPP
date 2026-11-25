#ifndef SETTINGS_H
#define SETTINGS_H

#include <cstddef>
#include <cstdint>

// ==============================================================================
// 1. SYSTEM CONFIGURATION & BUFFER SIZES
// ==============================================================================
constexpr size_t BLOCK_SIZE = 48; // Safe block size: matches libDaisy Patch SM default
static_assert(BLOCK_SIZE <= 48, "BLOCK_SIZE > 48 risks STM32 interrupt stack overflow");

constexpr bool ENABLE_DIAGNOSTICS = false; // Set true to print 2 Hz telemetry over USB CDC
constexpr float SAMPLE_RATE = 48000.0f;
constexpr size_t BUFFER_SAMPLE_RATE = 48000;
constexpr size_t MAX_BUFFER_SIZE = 96000; // 2.0 seconds @ 48 kHz in internal AXI SRAM

// ==============================================================================
// 2. HARDWARE CONTROL TIMING & SMOOTHING
// ==============================================================================
constexpr uint32_t CONTROLS_UPDATE_INTERVAL_MS = 10;   // 100 Hz analog/CV reads & CV out
constexpr uint32_t DIGITAL_CONTROLS_INTERVAL_HZ = 1000; // 1000 Hz switch/gate polling & LED PWM
constexpr uint32_t SERIAL_DIAGNOSTICS_MS = 500;         // 2 Hz when diagnostics enabled

constexpr float IO_SMOOTHING_COEFFICIENT = 0.15f; // EMA filter smoothing for knobs & CVs

// ==============================================================================
// 3. GRANULAR PROCESSOR SETTINGS
// ==============================================================================
constexpr int MAX_ACTIVE_GRAINS = 32;
constexpr int MIN_GRAIN_PLAYS = 2;
constexpr float GRAIN_SIZE_MIN = 0.10666667f; // 80 ms @ 48 kHz (3840 / 36000 samples)
constexpr float GRAIN_SIZE_MAX = 1.0f;        // 750 ms per pass (1.5 s per 2-pass voice) @ 48 kHz
constexpr float GRAIN_WINDOW_TAPER = 0.70f;   // Tukey window taper
constexpr float GRAIN_MIN_START_JITTER_SAMPLES = 960.0f;
constexpr float GRAIN_MAX_LIVE_JITTER_SAMPLES = 24000.0f; // Up to 500 ms live playhead time-scatter
constexpr float RECORD_FEEDBACK = 0.20f;
constexpr float FREEZE_OVERDUB_FEEDBACK = 0.95f;
constexpr float OVERDUB_FEEDBACK_SLEW_MS = 50.0f;
constexpr int RESUME_CROSSFADE_SAMPLES = 96;
constexpr float RANDOM_DIRECTION_REVERSE_CHANCE = 0.5f; // Switch 2 CENTER: per-grain coin flip

// ==============================================================================
// 4. FILTER & TONE SHAPING (Chamberlin SVF Bank)
// ==============================================================================
constexpr float FILTER_MIN_FREQ = 120.0f;
constexpr float FILTER_MAX_FREQ = 5500.0f;
constexpr float FILTER_MIN_RESONANCE = 0.08f;
constexpr float FILTER_MAX_RESONANCE = 0.20f;
constexpr float FILTER_RES_LFO_AMOUNT = 0.04f;
constexpr float FILTER_RES_CEILING = 0.25f; // Unconditional stability damping bound
constexpr float FILTER_VCA_FLOOR = 0.55f;
constexpr float HIGH_FREQ_DAMPING = 0.08f;

constexpr bool ENABLE_PRE_FILTER_BANK = true;
constexpr bool ENABLE_POST_FILTER_BANK = false;
constexpr bool ENABLE_ALLPASS_FILTER_BANK = true;
constexpr float FILTER_BANK_MIN_RANGE = 0.001f;

// ==============================================================================
// 5. REVERB, DIFFUSION & MASTER DYNAMICS
// ==============================================================================
constexpr float REVERB_STEREO_WIDTH = 1.5f; // 1.5x mid/side widening
constexpr float MAX_DIFFUSION_AMOUNT = 1.0f;
constexpr float DIFFUSION_TANK_MAX_FEEDBACK = 0.985f; // Expansive ambient pad decays
constexpr bool ENABLE_PRE_DELAY = false;
constexpr float REVERB_CROSS_FEEDBACK = 0.10f;
constexpr float MASTER_OUTPUT_GAIN = 2.0f;
constexpr float REVERB_OUTPUT_GAIN = 0.86f;
constexpr float BYPASS_CROSSFADE_MS = 10.0f;

// Master bus dynamics
constexpr float MASTER_COMP_THRESHOLD_DB = -3.0f;
constexpr float MASTER_COMP_RATIO = 4.0f;
constexpr float MASTER_COMP_ATTACK_S = 0.005f;
constexpr float MASTER_COMP_RELEASE_S = 0.080f;
constexpr float OUTPUT_LIMIT_KNEE = 0.80f; // Rational output ceiling knee

// ==============================================================================
// 6. TAPE MODULATION & REVERB TANK GEOMETRY
// ==============================================================================
constexpr float TAPE_WARBLE_RATE = 0.15f;  // Hz
constexpr float TAPE_WARBLE_DEPTH = 48.0f; // Samples (+/- 1ms at 48k)
constexpr float TANK_DELAY_L_S = 0.047f;  // 47 ms
constexpr float TANK_DELAY_R_S = 0.073f;  // 73 ms
constexpr size_t TANK_DELAY_MAX_SAMPLES = 4096;

// ==============================================================================
// 7. RHYTHM & MODULATION
// ==============================================================================
constexpr float DEFAULT_BPM = 120.0f;
constexpr float LFO_RATE_MULTIPLIER = 2.5f;
constexpr float LFO_TUNED_RATE_SCALE = 1.0f / 64.0f;
constexpr float FILTER_LFO_RATE_SCALE = 4.0f * LFO_TUNED_RATE_SCALE; // 2.5s..25s polyrhythmic filter sweeps

constexpr uint32_t STARTUP_LED_STEP_MS = 45;

constexpr bool PER_GRAIN_ARP_PITCH = true;
constexpr float ARP_PITCH_MIN_ST = -12.0f;
constexpr float ARP_PITCH_MAX_ST = 24.0f;
constexpr float GRAIN_DETUNE_MIN_ST = 0.04f;
constexpr float GRAIN_DETUNE_MAX_ST = 0.15f;

// ==============================================================================
// 8. ESTUARY CV MODULATION DEPTHS
// ==============================================================================
constexpr float CV_MOD_SIZE_DEPTH = 0.50f;
constexpr float CV_MOD_DENSITY_DEPTH = 0.50f;
constexpr float CV_MOD_ARP_DEPTH = 0.50f;
constexpr float CV_MOD_SPACE_FILTER_DEPTH = 0.35f;
constexpr float CV_MOD_SPACE_FEEDBACK_DEPTH = 0.50f;

#endif // SETTINGS_H
