#include "mock_libdaisy.h"
#include "daisysp.h"

#include "../AudioEngine.h"
#include "../Diffuser.h"
#include "../EstuaryHardwareInterface.h"
#include "../GranularBuffer.h"
#include "../RealtimeGranulator.h"
#include "../Settings.h"
#include "../State.h"
#include "../StereoInputSummer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <vector>

using namespace landscape;

// ============================================================================
// Test Framework Macros & Utilities
// ============================================================================
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
  do { \
    if (!(cond)) { \
      std::fprintf(stderr, "FAIL: %s (line %d): %s\n", __FUNCTION__, __LINE__, msg); \
      g_tests_failed++; \
      return; \
    } \
  } while (0)

#define TEST_ASSERT_NEAR(a, b, tol, msg) \
  do { \
    if (std::fabs((a) - (b)) > (tol)) { \
      std::fprintf(stderr, "FAIL: %s (line %d): %s (expected %.6f, got %.6f, tol %.6f)\n", \
                   __FUNCTION__, __LINE__, msg, static_cast<double>(b), static_cast<double>(a), static_cast<double>(tol)); \
      g_tests_failed++; \
      return; \
    } \
  } while (0)

#define RUN_TEST(fn) \
  do { \
    int prev_failed = g_tests_failed; \
    fn(); \
    if (g_tests_failed == prev_failed) { \
      g_tests_passed++; \
      std::printf("PASS: %s\n", #fn); \
    } \
  } while (0)

static float CalculateRms(const float *buf, size_t n) {
  if (n == 0) return 0.0f;
  double sum = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sum += static_cast<double>(buf[i]) * static_cast<double>(buf[i]);
  }
  return static_cast<float>(std::sqrt(sum / static_cast<double>(n)));
}

static float ComputeGoertzelAmplitude(const float *data, size_t n, float target_freq, float sr) {
  float omega = 2.0f * 3.14159265358979323846f * target_freq / sr;
  float cosine = std::cos(omega);
  float coeff = 2.0f * cosine;

  float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    q0 = coeff * q1 - q2 + data[i];
    q2 = q1;
    q1 = q0;
  }

  float real = q1 - q2 * cosine;
  float imag = q2 * std::sin(omega);
  return 2.0f * std::sqrt(real * real + imag * imag) / static_cast<float>(n);
}

static void RunAudioBlocks(AudioEngine &engine, GranulatorState &state,
                           const std::vector<float> &in_l, const std::vector<float> &in_r,
                           std::vector<float> &out_l, std::vector<float> &out_r,
                           size_t block_size = BLOCK_SIZE) {
  size_t total = in_l.size();
  out_l.assign(total, 0.0f);
  out_r.assign(total, 0.0f);
  size_t pos = 0;
  while (pos < total) {
    size_t n = std::min(block_size, total - pos);
    float *in_ptrs[2] = { const_cast<float *>(in_l.data() + pos), const_cast<float *>(in_r.data() + pos) };
    float *out_ptrs[2] = { out_l.data() + pos, out_r.data() + pos };
    engine.ProcessBlock(in_ptrs, out_ptrs, n, state);
    pos += n;
  }
}

// ============================================================================
// SECTION 1: Estuary Option A Hardware Interface Tests
// ============================================================================

static void test_estuary_switches_and_gates() {
  ccam::hw::Estuary estuary;
  estuary.Init();
  EstuaryHardwareInterface hw;
  hw.Init();
  GranulatorState state;

  // Switch 1: Arp Scale (1=FIFTHS, 0=CHROMATIC, 2=CUSTOM)
  estuary.switches[0].Set(1);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.arpMode == ::ArpMode::FIFTHS, "Switch 1 UP should select FIFTHS");

  estuary.switches[0].Set(0);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.arpMode == ::ArpMode::CHROMATIC, "Switch 1 CENTER should select CHROMATIC");

  estuary.switches[0].Set(2);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.arpMode == ::ArpMode::CUSTOM, "Switch 1 DOWN should select CUSTOM");

  // Switch 2: Grain Direction (1=FORWARD, 0=RANDOM, 2=REVERSE)
  estuary.switches[1].Set(1);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.grainDirection == GrainDirection::FORWARD, "Switch 2 UP should select FORWARD");

  estuary.switches[1].Set(0);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.grainDirection == GrainDirection::RANDOM, "Switch 2 CENTER should select RANDOM");

  estuary.switches[1].Set(2);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.grainDirection == GrainDirection::REVERSE, "Switch 2 DOWN should select REVERSE");

  // Gate 1 = Freeze, Gate 2 = Overdub
  estuary.GateIn1().SetState(false);
  estuary.GateIn2().SetState(false);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(!state.isFrozen && !state.gate1Freeze && !state.gate2Overdub, "Idle gates should unfreeze");

  // Gate 2 HIGH -> Overdub Mode
  estuary.GateIn2().SetState(true);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.isFrozen && state.freezeMode == FreezeMode::OVERDUB && state.gate2Overdub,
              "Gate 2 HIGH should engage FreezeMode::OVERDUB");

  // Both Gate 1 & Gate 2 HIGH -> Freeze takes priority
  estuary.GateIn1().SetState(true);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.isFrozen && state.freezeMode == FreezeMode::MOMENTARY && state.gate1Freeze,
              "Gate 1 Freeze should take priority over Gate 2 Overdub when both HIGH");

  // Release Gate 1 while Gate 2 stays HIGH -> returns to Overdub
  estuary.GateIn1().SetState(false);
  hw.ProcessDigitalControls(estuary, state);
  TEST_ASSERT(state.isFrozen && state.freezeMode == FreezeMode::OVERDUB,
              "Releasing Gate 1 with Gate 2 HIGH should restore Overdub");
}

static void test_estuary_knobs_and_bipolar_cv_mapping() {
  ccam::hw::Estuary estuary;
  estuary.Init();
  EstuaryHardwareInterface hw;
  hw.Init();
  GranulatorState state;

  // Set all 8 knobs to known values and CVs to 0
  estuary.SetKnob(0, 0.25f); // Size
  estuary.SetKnob(1, 0.50f); // Density
  estuary.SetKnob(2, 0.40f); // Arp
  estuary.SetKnob(3, 0.70f); // Random
  estuary.SetKnob(4, 0.60f); // Filter/Diffusion
  estuary.SetKnob(5, 0.80f); // Reverb Feedback
  estuary.SetKnob(6, 0.30f); // Verb Mix
  estuary.SetKnob(7, 0.90f); // Dry/Wet

  // First read seeds EMA immediately (no startup sweep from 0.0)
  hw.ProcessAnalogControls(estuary, state);
  TEST_ASSERT_NEAR(state.grainSize, 0.25f, 1e-5f, "First read should seed grainSize");
  TEST_ASSERT_NEAR(state.grainDensity, 0.50f, 1e-5f, "First read should seed grainDensity");
  TEST_ASSERT_NEAR(state.grainArp, 0.40f, 1e-5f, "First read should seed grainArp");
  TEST_ASSERT_NEAR(state.randomness, 0.70f, 1e-5f, "First read should seed randomness");
  TEST_ASSERT_NEAR(state.filterDiffusion, 0.60f, 1e-5f, "First read should seed filterDiffusion");
  TEST_ASSERT_NEAR(state.filterRange, std::pow(0.60f, 0.70f), 1e-5f, "filterRange power 0.70 curve");
  TEST_ASSERT_NEAR(state.diffusionAmount, 0.60f, 1e-5f, "diffusionAmount follows Knob 4");
  TEST_ASSERT_NEAR(state.verbMix, 0.30f, 1e-5f, "verbMix follows Knob 6 independently of Knob 5");
  TEST_ASSERT_NEAR(state.dryWetMix, 0.90f, 1e-5f, "dryWetMix follows Knob 7");
  float expected_fb = state.feedbackMin + 0.80f * (state.feedbackMax - state.feedbackMin);
  TEST_ASSERT_NEAR(state.verbFeedback, expected_fb, 1e-5f, "verbFeedback follows Knob 5 independently");

  // Apply bipolar CVs (+1.0 and -1.0) and converge EMA
  estuary.SetCv(0, +1.0f); // +0.50 to Size -> 0.75
  estuary.SetCv(1, -0.5f); // -0.25 to Density -> 0.25
  estuary.SetCv(2, +2.0f); // Out-of-range CV clamped to +1.0 -> +0.50 to Arp -> 0.90
  estuary.SetCv(3, -1.0f); // -0.35 to FilterDiffusion -> 0.25; -0.50 to Feedback -> 0.30

  for (int i = 0; i < 100; ++i) {
    hw.ProcessAnalogControls(estuary, state);
  }
  TEST_ASSERT_NEAR(state.grainSize, 0.75f, 1e-4f, "CV1 should modulate grainSize");
  TEST_ASSERT_NEAR(state.grainDensity, 0.25f, 1e-4f, "CV2 bipolar negative should reduce grainDensity");
  TEST_ASSERT_NEAR(state.grainArp, 0.90f, 1e-4f, "CV3 clamped bipolar should modulate grainArp");
  TEST_ASSERT_NEAR(state.filterDiffusion, 0.25f, 1e-4f, "CV4 should modulate filterDiffusion");
  TEST_ASSERT_NEAR(state.masterFeedback, 0.30f, 1e-4f, "CV4 should modulate masterFeedback");
}

static void test_estuary_leds_and_cv_gate_outputs() {
  ccam::hw::Estuary estuary;
  estuary.Init();
  EstuaryHardwareInterface hw;
  hw.Init();
  GranulatorState state;

  state.grainSize = 0.0f;
  state.grainDensity = 1.0f;
  state.verbMix = 0.2f;
  state.dryWetMix = 0.2f;
  state.gate1Freeze = true;
  state.gate2Overdub = false;

  // Verify Freeze forces LED 6 to 1.0f and respects 0.0 / 1.0 bounds
  hw.UpdateLEDs(estuary, state);
  TEST_ASSERT_NEAR(estuary.leds[6].Get(), 1.0f, 1e-5f, "LED 6 should lock solid ON (1.0) when Freeze active");
  TEST_ASSERT_NEAR(estuary.leds[1].Get(), 1.0f, 1e-5f, "LED 1 at 1.0 should be 1.0");
  TEST_ASSERT_NEAR(estuary.leds[0].Get(), 0.0f, 1e-5f, "LED 0 at 0.0 should stay OFF (0.0)");

  // Verify Overdub forces LED 7 to 1.0f
  state.gate1Freeze = false;
  state.gate2Overdub = true;
  hw.UpdateLEDs(estuary, state);
  TEST_ASSERT_NEAR(estuary.leds[7].Get(), 1.0f, 1e-5f, "LED 7 should lock solid ON (1.0) when Overdub active");

  // Verify CV Out 1/2 (0..5V) and Gate Out 1/2
  state.gate1Freeze = true;
  state.gate2Overdub = false;
  hw.UpdateOutputs(estuary, state, 0.40f);
  TEST_ASSERT(estuary.GetGateOut1() == true, "GateOut1 should mirror Freeze");
  TEST_ASSERT(estuary.GetGateOut2() == false, "GateOut2 should mirror Overdub");
  TEST_ASSERT_NEAR(estuary.GetCvOut(2), 2.00f, 1e-4f, "CV Out 2 should output panLfo * 5V");
}

// ============================================================================
// SECTION 2: Smart L / Stereo Input Summer Tests
// ============================================================================

static void test_smart_stereo_input_summer_and_dry_passthrough() {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.dryWetMix = 0.0f; // Pure dry to measure unity & stereo pass-through cleanly
  state.verbMix = 0.0f;
  state.bypass = false;

  // Case 1: Mono L-only (In R is silent / unpatched) -> Zero -6 dB drop, dual-mono output
  const size_t n_mono = 4800; // 100 ms
  std::vector<float> in_l(n_mono), in_r(n_mono, 0.0f), out_l, out_r;
  for (size_t i = 0; i < n_mono; ++i) {
    in_l[i] = 0.25f * std::sin(2.0f * 3.14159265f * 440.0f * static_cast<float>(i) / SAMPLE_RATE);
  }
  RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);

  float rms_in = CalculateRms(in_l.data() + 1000, n_mono - 1000);
  float rms_out_l = CalculateRms(out_l.data() + 1000, n_mono - 1000);
  float rms_out_r = CalculateRms(out_r.data() + 1000, n_mono - 1000);
  TEST_ASSERT_NEAR(rms_out_l, rms_in, 0.01f, "Mono L-only dry output L should be unity (no -6 dB drop)");
  TEST_ASSERT_NEAR(rms_out_r, rms_in, 0.01f, "Mono L-only dry output R should mirror L");
  TEST_ASSERT_NEAR(engine->GetStereoInputMix(), 0.0f, 1e-5f, "StereoMix should stay 0 when In R is silent");

  // Case 2: Signal patched into In R (distinct 880 Hz tone on R, 440 Hz on L) -> crossfades to true stereo dry
  const size_t n_stereo = 9600; // 200 ms (> 50 ms crossfade)
  in_l.resize(n_stereo);
  in_r.resize(n_stereo);
  for (size_t i = 0; i < n_stereo; ++i) {
    in_l[i] = 0.25f * std::sin(2.0f * 3.14159265f * 440.0f * static_cast<float>(i) / SAMPLE_RATE);
    in_r[i] = 0.25f * std::sin(2.0f * 3.14159265f * 880.0f * static_cast<float>(i) / SAMPLE_RATE);
  }
  RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);
  TEST_ASSERT_NEAR(engine->GetStereoInputMix(), 1.0f, 1e-4f, "StereoMix should reach 1.0 when In R has signal");

  // Check spectral separation in the final 4096 samples (L has 440 Hz, R has 880 Hz)
  float l_440 = ComputeGoertzelAmplitude(out_l.data() + n_stereo - 4096, 4096, 440.0f, SAMPLE_RATE);
  float l_880 = ComputeGoertzelAmplitude(out_l.data() + n_stereo - 4096, 4096, 880.0f, SAMPLE_RATE);
  float r_440 = ComputeGoertzelAmplitude(out_r.data() + n_stereo - 4096, 4096, 440.0f, SAMPLE_RATE);
  float r_880 = ComputeGoertzelAmplitude(out_r.data() + n_stereo - 4096, 4096, 880.0f, SAMPLE_RATE);
  TEST_ASSERT(l_440 > 0.20f && l_880 < 0.01f, "Dry Out L should carry only In L (440 Hz)");
  TEST_ASSERT(r_880 > 0.20f && r_440 < 0.01f, "Dry Out R should carry only In R (880 Hz)");
}

// ============================================================================
// SECTION 3: Full DSP Engine Stability, Harmonic Prominence & RT60 Tests
// ============================================================================

static void test_dsp_stability_and_output_ceiling() {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = 0.8f;
  state.grainDensity = 1.0f;
  state.grainArp = 0.9f;
  state.randomness = 1.0f;
  state.filterDiffusion = 1.0f;
  state.filterRange = 1.0f;
  state.diffusionAmount = 1.0f;
  state.verbFeedback = 0.985f;
  state.verbMix = 1.0f;
  state.dryWetMix = 1.0f;
  state.grainDirection = GrainDirection::RANDOM;

  // Test arbitrary chunk sizes (32, 48, 64, 128) with loud impulse + hot sine burst
  const size_t chunk_sizes[] = {32, 48, 64, 128};
  for (size_t c = 0; c < 4; ++c) {
    size_t n_samples = 24000; // 0.5 s per chunk size
    std::vector<float> in_l(n_samples, 0.0f), in_r(n_samples, 0.0f), out_l, out_r;
    in_l[0] = 2.5f; // Hot transient
    for (size_t i = 1; i < 12000; ++i) {
      in_l[i] = 0.9f * std::sin(2.0f * 3.14159265f * 330.0f * static_cast<float>(i) / SAMPLE_RATE);
      in_r[i] = 0.9f * std::sin(2.0f * 3.14159265f * 495.0f * static_cast<float>(i) / SAMPLE_RATE);
    }
    RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r, chunk_sizes[c]);

    for (size_t i = 0; i < n_samples; ++i) {
      TEST_ASSERT(std::isfinite(out_l[i]) && std::isfinite(out_r[i]), "Output must never be NaN or Inf");
      TEST_ASSERT(std::fabs(out_l[i]) <= 1.0f && std::fabs(out_r[i]) <= 1.0f,
                  "Output ceiling must stay bounded within [-1.0, 1.0]");
    }
  }
}

static void test_goertzel_harmonic_prominence_all_arp_modes() {
  const ::ArpMode modes[] = {::ArpMode::FIFTHS, ::ArpMode::CHROMATIC, ::ArpMode::CUSTOM};
  for (int m = 0; m < 3; ++m) {
    auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
    engine->Init(SAMPLE_RATE);

    GranulatorState state;
    state.grainSize = 0.40f;
    state.grainDensity = 0.90f;
    state.grainArp = 1.0f;
    state.randomness = 0.0f;
    state.filterDiffusion = 0.50f;
    state.filterRange = std::pow(0.50f, 0.70f);
    state.diffusionAmount = 0.50f;
    state.verbFeedback = 0.30f;
    state.verbMix = 0.0f;
    state.dryWetMix = 0.50f;
    state.arpMode = modes[m];

    const size_t total = 96000; // 2.0 s so arp LFOs traverse scale steps
    std::vector<float> in_l(total), in_r(total, 0.0f), out_l, out_r;
    const float f0 = 220.0f;
    for (size_t i = 0; i < total; ++i) {
      in_l[i] = 0.30f * std::sin(2.0f * 3.14159265f * f0 * static_cast<float>(i) / SAMPLE_RATE);
    }
    RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);

    // Measure shifted harmonic energy across octaves/fifths/fourths/thirds/sixths/ninths
    const size_t win = 16384;
    const float *tail = out_l.data() + (total - win);
    const float harmonic_freqs[] = {
        f0 * 0.5f,                           // -12 st
        f0 * std::pow(2.0f, 4.0f / 12.0f),   // +4 st (Maj 3rd)
        f0 * std::pow(2.0f, 5.0f / 12.0f),   // +5 st (4th)
        f0 * std::pow(2.0f, 7.0f / 12.0f),   // +7 st (5th)
        f0 * std::pow(2.0f, 9.0f / 12.0f),   // +9 st (Maj 6th)
        f0 * 2.0f,                           // +12 st (Octave)
        f0 * std::pow(2.0f, 14.0f / 12.0f),  // +14 st (Maj 9th)
        f0 * std::pow(2.0f, 19.0f / 12.0f),  // +19 st (Octave + 5th)
        f0 * 4.0f                            // +24 st (Double Octave)
    };
    float amp_f0 = ComputeGoertzelAmplitude(tail, win, f0, SAMPLE_RATE);
    float harmonic_peak = 0.0f;
    for (float hf : harmonic_freqs) {
      harmonic_peak = std::max(harmonic_peak, ComputeGoertzelAmplitude(tail, win, hf, SAMPLE_RATE));
    }

    TEST_ASSERT(amp_f0 > 0.02f, "Fundamental must remain audible");
    TEST_ASSERT(harmonic_peak > amp_f0 * 0.15f,
                "Shifted granular harmonics must be clearly audible across all Arp modes");
  }
}

static void test_reverb_rt60_ambient_decay_and_overdub_loop() {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = 0.6f;
  state.grainDensity = 0.7f;
  state.grainArp = 0.3f;
  state.randomness = 0.1f;
  state.filterDiffusion = 1.0f;
  state.filterRange = 1.0f;
  state.diffusionAmount = 1.0f;
  state.verbFeedback = 0.985f; // Max ambient decay
  state.verbMix = 1.0f;
  state.dryWetMix = 1.0f;

  // Feed 0.5 s chord burst, then 12.0 s silence and verify tail remains clean & alive
  const size_t burst_n = 24000;
  const size_t tail_n = 12 * 48000;
  std::vector<float> in_l(burst_n + tail_n, 0.0f), in_r(burst_n + tail_n, 0.0f), out_l, out_r;
  for (size_t i = 0; i < burst_n; ++i) {
    in_l[i] = 0.35f * std::sin(2.0f * 3.14159265f * 261.63f * static_cast<float>(i) / SAMPLE_RATE);
  }
  RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);

  float rms_1s = CalculateRms(out_l.data() + burst_n + 48000, 48000);
  float rms_10s = CalculateRms(out_l.data() + burst_n + 10 * 48000, 48000);
  TEST_ASSERT(rms_1s > 0.01f, "Reverb tail at 1s should have strong energy");
  TEST_ASSERT(rms_10s > 0.001f && rms_10s < rms_1s,
              "Reverb tail at 10s should still be audibly decaying without muting or blowing up");

  // Also test Overdub Mode (Gate 2 HIGH): verify continuous 95% feedback sustain without clipping
  engine->Init(SAMPLE_RATE);
  state.verbMix = 0.0f;
  state.dryWetMix = 1.0f;
  state.grainArp = 0.0f;
  state.randomness = 0.0f;
  state.isFrozen = true;
  state.freezeMode = FreezeMode::OVERDUB;
  state.gate2Overdub = true;

  const size_t ovd_total = 4 * 48000; // 4 seconds (2 full buffer loops)
  std::vector<float> ovd_in_l(ovd_total, 0.0f), ovd_in_r(ovd_total, 0.0f), ovd_out_l, ovd_out_r;
  for (size_t i = 0; i < 48000; ++i) {
    ovd_in_l[i] = 0.4f * std::sin(2.0f * 3.14159265f * 330.0f * static_cast<float>(i) / SAMPLE_RATE);
  }
  RunAudioBlocks(*engine, state, ovd_in_l, ovd_in_r, ovd_out_l, ovd_out_r);

  float ovd_rms_3s = CalculateRms(ovd_out_l.data() + 3 * 48000, 48000);
  TEST_ASSERT(ovd_rms_3s > 0.05f, "Overdub mode (95% feedback) must sustain looped audio across buffer wraps");
  for (size_t i = 0; i < ovd_total; ++i) {
    TEST_ASSERT(std::isfinite(ovd_out_l[i]) && std::fabs(ovd_out_l[i]) <= 1.0f,
                "Overdub output must remain bounded and NaN-free");
  }
}

// ============================================================================
// SECTION 4: Grain Playback Direction (Switch 2)
// ============================================================================

struct DirectionRun {
  int forward_spawns = 0;
  int reverse_spawns = 0;
  float wet_rms = 0.0f;
  bool bounded = true;
};

// Renders 2 s of a 220 Hz tone fully wet and counts grain spawns per direction.
static DirectionRun RunDirection(GrainDirection dir, float grain_size, float grain_density) {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = grain_size;
  state.grainDensity = grain_density;
  state.grainArp = 0.5f;
  state.randomness = 1.0f; // Randomness no longer affects direction
  state.verbMix = 0.0f;
  state.dryWetMix = 1.0f;
  state.grainDirection = dir;

  const size_t total = 2 * 48000;
  std::vector<float> in_l(total), in_r(total, 0.0f), out_l(total), out_r(total);
  for (size_t i = 0; i < total; ++i) {
    in_l[i] = 0.3f * std::sin(2.0f * 3.14159265f * 220.0f * static_cast<float>(i) / SAMPLE_RATE);
  }

  // A fresh grain is the only voice state with lifespan == MIN_GRAIN_PLAYS (repeat passes drop it to 1).
  DirectionRun run;
  bool prev_active[landscape::MAX_VOICES] = {false};
  int prev_lifespan[landscape::MAX_VOICES] = {0};
  int prev_age[landscape::MAX_VOICES] = {0};
  for (size_t pos = 0; pos < total; pos += BLOCK_SIZE) {
    float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
    float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
    engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
    for (int v = 0; v < landscape::MAX_VOICES; ++v) {
      const auto &voice = engine->realtimeGranulator.GetVoice(v);
      const bool spawned = voice.active && voice.lifespan == MIN_GRAIN_PLAYS &&
          (!prev_active[v] || prev_lifespan[v] != MIN_GRAIN_PLAYS || voice.age < prev_age[v]);
      prev_active[v] = voice.active;
      prev_lifespan[v] = voice.lifespan;
      prev_age[v] = voice.age;
      if (spawned) {
        if (voice.rate < 0.0f) run.reverse_spawns++; else run.forward_spawns++;
      }
    }
  }

  for (size_t i = 0; i < total; ++i) {
    if (!std::isfinite(out_l[i]) || std::fabs(out_l[i]) > 1.0f) run.bounded = false;
  }
  run.wet_rms = CalculateRms(out_l.data() + 48000, 48000);
  return run;
}

static void test_grain_direction_modes() {
  // Direction per switch position: short dense grains give enough spawns to measure the split.
  DirectionRun fwd = RunDirection(GrainDirection::FORWARD, 0.3f, 1.0f);
  DirectionRun rnd = RunDirection(GrainDirection::RANDOM, 0.3f, 1.0f);
  DirectionRun rev = RunDirection(GrainDirection::REVERSE, 0.3f, 1.0f);
  TEST_ASSERT(fwd.reverse_spawns == 0 && fwd.forward_spawns > 0,
              "FORWARD must spawn only forward grains, even at max Randomness");
  TEST_ASSERT(rev.forward_spawns == 0 && rev.reverse_spawns > 0, "REVERSE must spawn only reversed grains");
  const float rev_share = static_cast<float>(rnd.reverse_spawns) /
                          static_cast<float>(rnd.reverse_spawns + rnd.forward_spawns);
  TEST_ASSERT(rnd.reverse_spawns + rnd.forward_spawns > 100, "RANDOM run should spawn enough grains to measure");
  TEST_ASSERT(rev_share > 0.40f && rev_share < 0.60f, "RANDOM should reverse roughly half the grains");

  // Level match at the longest grains, where reversed grains used to be cut short (~6 dB quieter).
  const GrainDirection dirs[] = {GrainDirection::FORWARD, GrainDirection::RANDOM, GrainDirection::REVERSE};
  float wet_rms[3];
  for (int d = 0; d < 3; ++d) {
    DirectionRun run = RunDirection(dirs[d], 1.0f, 0.6f);
    TEST_ASSERT(run.bounded, "Output must stay bounded and finite in every direction");
    TEST_ASSERT(run.wet_rms > 0.02f, "Wet grains must stay audible in every direction");
    wet_rms[d] = run.wet_rms;
  }
  for (int d = 1; d < 3; ++d) {
    TEST_ASSERT_NEAR(20.0f * std::log10(wet_rms[d] / wet_rms[0]), 0.0f, 3.0f,
                     "RANDOM and REVERSE must stay within 3 dB of FORWARD at max grain size");
  }
}

// ============================================================================
// SECTION 5: Loudness Consistency Across Size / Density / Freeze
// ============================================================================

// Wet granulator level (dB re input) for a steady chord, frozen after 1 s when requested.
static float RenderGranularLevelDb(float grain_size, float grain_density, bool freeze) {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = grain_size;
  state.grainDensity = grain_density;
  state.grainArp = 0.0f;
  state.randomness = 0.0f; // Smallest freeze window (0.25 s): the hardest case for long grains
  state.verbMix = 0.0f;
  state.dryWetMix = 1.0f;

  const size_t total = 5 * 48000;
  const size_t meas = 2 * 48000;
  std::vector<float> in_l(total), in_r(total, 0.0f), out_l(total), out_r(total);
  for (size_t i = 0; i < total; ++i) {
    const float t = static_cast<float>(i) / SAMPLE_RATE;
    in_l[i] = 0.1f * (std::sin(2.0f * 3.14159265f * 130.81f * t) + std::sin(2.0f * 3.14159265f * 164.81f * t) +
                      std::sin(2.0f * 3.14159265f * 196.0f * t));
  }
  for (size_t pos = 0; pos < total; pos += BLOCK_SIZE) {
    state.isFrozen = freeze && pos >= 48000;
    state.gate1Freeze = state.isFrozen;
    float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
    float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
    engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
  }
  const size_t start = total - meas;
  const float in_rms = CalculateRms(in_l.data() + start, meas);
  const float out_l_rms = CalculateRms(out_l.data() + start, meas);
  const float out_r_rms = CalculateRms(out_r.data() + start, meas);
  const float out_rms = std::sqrt(0.5f * (out_l_rms * out_l_rms + out_r_rms * out_r_rms));
  return 20.0f * std::log10(out_rms / in_rms);
}

static void test_loudness_consistent_across_size_density_freeze() {
  // Before equal-power density trim and real-length spawn tracking, this spread was ~21 dB
  // (Freeze + max Size + min Density collapsed to ~0 active grains).
  float lo = 1e9f, hi = -1e9f;
  for (int fz = 0; fz < 2; ++fz) {
    for (float size : {0.1f, 1.0f}) {
      for (float density : {0.0f, 0.5f, 1.0f}) {
        const float db = RenderGranularLevelDb(size, density, fz != 0);
        TEST_ASSERT(std::isfinite(db), "Level must be finite");
        lo = std::min(lo, db);
        hi = std::max(hi, db);
      }
    }
  }
  TEST_ASSERT(hi - lo < 4.0f, "Wet level must stay within 4 dB across Size, Density and Freeze");
  TEST_ASSERT(lo > -3.0f && hi < 3.0f, "Wet level should sit within +/-3 dB of the input");
}

// Fully wet level (dB re input) for a steady chord scaled to the given peak level.
static float RenderPresetWetLevelDb(float input_peak_dbfs, bool dense_wash = false) {
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = dense_wash ? 0.8f : 0.5f;
  state.grainDensity = dense_wash ? 1.0f : 0.5f;
  state.grainArp = dense_wash ? 0.6f : 0.5f;
  state.randomness = dense_wash ? 0.6f : 0.5f;
  state.filterDiffusion = dense_wash ? 1.0f : 0.5f;
  state.filterRange = std::pow(state.filterDiffusion, 0.70f);
  state.diffusionAmount = state.filterDiffusion;
  state.verbFeedback = dense_wash
      ? state.feedbackMax
      : (state.feedbackMin + 0.5f * (state.feedbackMax - state.feedbackMin));
  state.verbMix = dense_wash ? 1.0f : 0.5f;
  state.dryWetMix = 1.0f;

  const size_t total = 6 * 48000;
  const size_t meas = 3 * 48000;
  std::vector<float> in_l(total), in_r(total, 0.0f), out_l(total), out_r(total);
  float peak = 0.0f;
  for (size_t i = 0; i < total; ++i) {
    const float t = static_cast<float>(i) / SAMPLE_RATE;
    float x = 0.0f;
    for (float f : {130.81f, 164.81f, 196.0f, 261.63f}) {
      x += std::sin(2.0f * 3.14159265f * f * t) + 0.3f * std::sin(2.0f * 3.14159265f * 2.0f * f * t);
    }
    in_l[i] = x;
    peak = std::max(peak, std::fabs(x));
  }
  const float scale = std::pow(10.0f, input_peak_dbfs / 20.0f) / peak;
  for (float &x : in_l) x *= scale;
  RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);

  const size_t start = total - meas;
  const float in_rms = CalculateRms(in_l.data() + start, meas);
  const float l = CalculateRms(out_l.data() + start, meas);
  const float r = CalculateRms(out_r.data() + start, meas);
  return 20.0f * std::log10(std::sqrt(0.5f * (l * l + r * r)) / in_rms);
}

static void test_wet_gain_independent_of_input_level() {
  // The filter bank's level compensation must stay engaged down to -48 dBFS peak input
  // on both centered knobs and full filter/reverb dense-wash settings.
  const float db_quiet = RenderPresetWetLevelDb(-48.0f, false);
  const float db_normal = RenderPresetWetLevelDb(-18.0f, false);
  TEST_ASSERT(std::isfinite(db_quiet) && std::isfinite(db_normal), "Centered levels must be finite");
  TEST_ASSERT_NEAR(db_quiet, db_normal, 0.5f, "Centered wet gain at -48 dBFS input must match -18 dBFS within 0.5 dB");

  const float wash_quiet = RenderPresetWetLevelDb(-48.0f, true);
  const float wash_normal = RenderPresetWetLevelDb(-18.0f, true);
  TEST_ASSERT(std::isfinite(wash_quiet) && std::isfinite(wash_normal), "Dense wash levels must be finite");
  TEST_ASSERT_NEAR(wash_quiet, wash_normal, 0.5f, "Dense wash wet gain at -48 dBFS input must match -18 dBFS within 0.5 dB");
}

static void test_sparse_window_stability_and_freeze_grain_length() {
  // 1. At Density=0 (1 active grain), constant-power panLfo + centered single-grain panning
  // must prevent the 14 dB grain-to-grain dropout lottery across 0.5 s windows.
  auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
  engine->Init(SAMPLE_RATE);

  GranulatorState state;
  state.grainSize = 0.5f;
  state.grainDensity = 0.0f;
  state.grainArp = 0.0f;
  state.randomness = 0.0f;
  state.filterDiffusion = 0.0f;
  state.filterRange = 0.0f;
  state.diffusionAmount = 0.0f;
  state.verbFeedback = 0.1f;
  state.verbMix = 0.0f;
  state.dryWetMix = 1.0f;

  const size_t total = 8 * 48000;
  const size_t meas_start = 2 * 48000;
  const size_t win = 24000; // 0.5 s window
  std::vector<float> in_l(total), in_r(total, 0.0f), out_l(total), out_r(total);
  for (size_t i = 0; i < total; ++i) {
    const float t = static_cast<float>(i) / SAMPLE_RATE;
    in_l[i] = 0.1f * (std::sin(2.0f * 3.14159265f * 130.81f * t) +
                      std::sin(2.0f * 3.14159265f * 164.81f * t) +
                      std::sin(2.0f * 3.14159265f * 196.0f * t));
  }
  RunAudioBlocks(*engine, state, in_l, in_r, out_l, out_r);

  float min_win_db = 1e9f, max_win_db = -1e9f;
  for (size_t p = meas_start; p + win <= total; p += win) {
    const float in_rms = CalculateRms(in_l.data() + p, win);
    const float l_rms = CalculateRms(out_l.data() + p, win);
    const float r_rms = CalculateRms(out_r.data() + p, win);
    const float out_rms = std::sqrt(0.5f * (l_rms * l_rms + r_rms * r_rms));
    const float db = 20.0f * std::log10(out_rms / in_rms);
    min_win_db = std::min(min_win_db, db);
    max_win_db = std::max(max_win_db, db);
  }
  TEST_ASSERT(max_win_db - min_win_db < 3.5f,
              "Sparse 1-voice 0.5s window levels must stay within 3.5 dB across panLfo sweep");

  // 2. In Freeze mode with max Grain Size (1.0) and min Randomness (0.0 -> 0.25s slice),
  // start positions must leave room for grain travel so no grain is chopped to <50 ms.
  state.grainSize = 1.0f;
  state.grainDensity = 0.5f;
  state.isFrozen = true;
  state.gate1Freeze = true;
  int min_spawned_len = 1000000;
  int spawns = 0;
  for (size_t pos = 0; pos < 48000; pos += BLOCK_SIZE) {
    float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
    float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
    engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
    for (int v = 0; v < landscape::MAX_VOICES; ++v) {
      const auto &voice = engine->realtimeGranulator.GetVoice(v);
      if (voice.active && voice.lifespan == MIN_GRAIN_PLAYS && voice.age <= static_cast<int>(BLOCK_SIZE)) {
        min_spawned_len = std::min(min_spawned_len, voice.length);
        spawns++;
      }
    }
  }
  TEST_ASSERT(spawns > 0 && min_spawned_len >= 7200,
              "Frozen max-size grains must never be chopped below 150 ms (7200 samples) near slice walls");
}

static void test_musical_arp_voicing_filter_routing_and_1s_grains() {
  // 1. At Arp Reach = 0% and Randomness = 100%, pitch must remain strictly unison + micro-detune
  // (no interval or octave jumps). Also verify live playhead jitter expands well beyond 960 samples.
  {
    auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
    engine->Init(SAMPLE_RATE);

    GranulatorState state;
    state.grainSize = 0.25f;
    state.grainDensity = 0.8f;
    state.grainArp = 0.0f;
    state.randomness = 1.0f;
    state.verbMix = 0.0f;
    state.dryWetMix = 1.0f;

    const size_t total = 3 * 48000;
    std::vector<float> in_l(total, 0.1f), in_r(total, 0.0f), out_l(total), out_r(total);
    float max_spawn_delay = 0.0f;
    int checked = 0;
    for (size_t pos = 0; pos < total; pos += BLOCK_SIZE) {
      float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
      float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
      engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
      for (int v = 0; v < landscape::MAX_VOICES; ++v) {
        const auto &voice = engine->realtimeGranulator.GetVoice(v);
        if (voice.active && voice.lifespan == MIN_GRAIN_PLAYS && voice.age <= static_cast<int>(BLOCK_SIZE)) {
          TEST_ASSERT(std::fabs(voice.rate - 1.0f) < 0.015f,
                      "Arp Reach = 0% must stay at unison + micro-detune even at max Randomness");
          if (pos >= 2 * 48000) {
            max_spawn_delay = std::max(max_spawn_delay, voice.start_delay_pos);
          }
          checked++;
        }
      }
    }
    TEST_ASSERT(checked > 20, "Should spawn sufficient grains to verify unison and jitter");
    TEST_ASSERT(max_spawn_delay > 10000.0f,
                "Max Randomness in live mode must expand playhead time-scatter well beyond 20 ms");
  }

  // 2. Across all 3 ArpModes at Arp Reach = 100% and Randomness = 100%, every grain pitch must
  // belong to a consonant scale degree (0, 2, 4, 5, 7, 9 st mod 12) — zero semitone or tritone clashes.
  {
    const ::ArpMode modes[3] = {::ArpMode::FIFTHS, ::ArpMode::CHROMATIC, ::ArpMode::CUSTOM};
    const float allowed_mod12[6] = {0.0f, 2.0f, 4.0f, 5.0f, 7.0f, 9.0f};
    bool saw_third_or_sixth_in_center = false;

    for (int m = 0; m < 3; ++m) {
      auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
      engine->Init(SAMPLE_RATE);

      GranulatorState state;
      state.grainSize = 0.2f;
      state.grainDensity = 1.0f;
      state.grainArp = 1.0f;
      state.randomness = 1.0f;
      state.arpMode = modes[m];

      const size_t total = 2 * 48000;
      std::vector<float> in_l(total, 0.1f), in_r(total, 0.0f), out_l(total), out_r(total);
      for (size_t pos = 0; pos < total; pos += BLOCK_SIZE) {
        float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
        float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
        engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
        for (int v = 0; v < landscape::MAX_VOICES; ++v) {
          const auto &voice = engine->realtimeGranulator.GetVoice(v);
          if (voice.active && voice.lifespan == MIN_GRAIN_PLAYS && voice.age <= static_cast<int>(BLOCK_SIZE)) {
            const float st = 12.0f * std::log2(std::fabs(voice.rate));
            TEST_ASSERT(st >= -12.25f && st <= 24.25f, "Pitch must stay folded within [-12, +24] st");
            float mod12 = std::fmod(st + 24.0f, 12.0f);
            if (mod12 < 0.0f) mod12 += 12.0f;
            float min_dist = 12.0f;
            for (float deg : allowed_mod12) {
              float d = std::fabs(mod12 - deg);
              d = std::min(d, 12.0f - d);
              min_dist = std::min(min_dist, d);
            }
            TEST_ASSERT(min_dist < 0.35f,
                        "Every spawned grain must land on a consonant scale degree (no semitone/tritone clash)");
            if (modes[m] == ::ArpMode::CHROMATIC &&
                (std::fabs(mod12 - 4.0f) < 0.30f || std::fabs(mod12 - 9.0f) < 0.30f || std::fabs(mod12 - 2.0f) < 0.30f)) {
              saw_third_or_sixth_in_center = true;
            }
          }
        }
      }
    }
    TEST_ASSERT(saw_third_or_sixth_in_center,
                "Switch 1 CENTER (Pentatonic/Add9) must produce 3rd/6th/9th harmonic intervals distinct from FIFTHS");
  }

  // 3. Max Grain Size (1.0 -> 750 ms = 36000 samples per pass, 1.5 s per 2-pass voice)
  {
    auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
    engine->Init(SAMPLE_RATE);

    GranulatorState state;
    state.grainSize = 1.0f;
    state.grainDensity = 0.0f;
    state.grainArp = 0.0f;
    state.randomness = 0.0f;

    const size_t total = 3 * 48000;
    std::vector<float> in_l(total, 0.1f), in_r(total, 0.0f), out_l(total), out_r(total);
    int max_len = 0;
    for (size_t pos = 0; pos < total; pos += BLOCK_SIZE) {
      float *in_ptrs[2] = {in_l.data() + pos, in_r.data() + pos};
      float *out_ptrs[2] = {out_l.data() + pos, out_r.data() + pos};
      engine->ProcessBlock(in_ptrs, out_ptrs, BLOCK_SIZE, state);
      if (pos >= 2 * 48000) {
        for (int v = 0; v < landscape::MAX_VOICES; ++v) {
          const auto &voice = engine->realtimeGranulator.GetVoice(v);
          if (voice.active) {
            max_len = std::max(max_len, voice.length);
          }
        }
      }
    }
    TEST_ASSERT(max_len >= 27000, "Max Grain Size must reach ~750 ms (36000 samples, >=27000 with min jitter) after buffer wrap");
  }

  // 4. Knob 4 (Filter / Diffusion) must actively shape the granular cloud even when VerbMix = 0%.
  {
    const size_t total = 4 * 48000;
    const size_t meas = 2 * 48000;
    std::vector<float> in_l(total), in_r(total, 0.0f);
    for (size_t i = 0; i < total; ++i) {
      const float t = static_cast<float>(i) / SAMPLE_RATE;
      in_l[i] = 0.15f * (std::sin(2.0f * 3.14159265f * 220.0f * t) +
                         std::sin(2.0f * 3.14159265f * 1760.0f * t) +
                         std::sin(2.0f * 3.14159265f * 3520.0f * t));
    }

    std::vector<float> out_f0_l, out_f0_r, out_f1_l, out_f1_r;
    {
      auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
      engine->Init(SAMPLE_RATE);
      GranulatorState s;
      s.grainSize = 0.5f; s.grainDensity = 0.5f; s.grainArp = 0.0f; s.randomness = 0.0f;
      s.filterDiffusion = 0.0f; s.filterRange = 0.0f; s.diffusionAmount = 0.0f;
      s.verbMix = 0.0f; s.dryWetMix = 1.0f;
      RunAudioBlocks(*engine, s, in_l, in_r, out_f0_l, out_f0_r);
    }
    {
      auto engine = std::unique_ptr<AudioEngine>(new AudioEngine());
      engine->Init(SAMPLE_RATE);
      GranulatorState s;
      s.grainSize = 0.5f; s.grainDensity = 0.5f; s.grainArp = 0.0f; s.randomness = 0.0f;
      s.filterDiffusion = 1.0f; s.filterRange = 1.0f; s.diffusionAmount = 1.0f;
      s.verbMix = 0.0f; s.dryWetMix = 1.0f;
      RunAudioBlocks(*engine, s, in_l, in_r, out_f1_l, out_f1_r);
    }

    float diff_rms = 0.0f;
    for (size_t i = total - meas; i < total; ++i) {
      const float d = out_f1_l[i] - out_f0_l[i];
      diff_rms += d * d;
    }
    diff_rms = std::sqrt(diff_rms / static_cast<float>(meas));
    const float rms_f0 = CalculateRms(out_f0_l.data() + total - meas, meas);
    const float rms_f1 = CalculateRms(out_f1_l.data() + total - meas, meas);
    TEST_ASSERT(diff_rms > 0.02f, "Knob 4 (Filter/Diffusion) must audibly filter the granular cloud even at VerbMix = 0%");
    TEST_ASSERT_NEAR(20.0f * std::log10(rms_f1 / rms_f0), 0.0f, 1.5f,
                     "Filter bank level compensation must keep VerbMix = 0% output within 1.5 dB when Filter is swept 0 -> 1");
  }
}

int main() {
  std::printf("Running Estuary Landscape Granulator Offline Test Suite...\n");

  RUN_TEST(test_estuary_switches_and_gates);
  RUN_TEST(test_estuary_knobs_and_bipolar_cv_mapping);
  RUN_TEST(test_estuary_leds_and_cv_gate_outputs);
  RUN_TEST(test_smart_stereo_input_summer_and_dry_passthrough);
  RUN_TEST(test_dsp_stability_and_output_ceiling);
  RUN_TEST(test_goertzel_harmonic_prominence_all_arp_modes);
  RUN_TEST(test_reverb_rt60_ambient_decay_and_overdub_loop);
  RUN_TEST(test_grain_direction_modes);
  RUN_TEST(test_loudness_consistent_across_size_density_freeze);
  RUN_TEST(test_wet_gain_independent_of_input_level);
  RUN_TEST(test_sparse_window_stability_and_freeze_grain_length);
  RUN_TEST(test_musical_arp_voicing_filter_routing_and_1s_grains);

  std::printf("\n====================================================================\n");
  std::printf("ESTUARY SUITE RESULTS: %d passed, %d failed.\n", g_tests_passed, g_tests_failed);
  std::printf("====================================================================\n");
  return (g_tests_failed == 0) ? 0 : 1;
}
