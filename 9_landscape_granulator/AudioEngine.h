#pragma once
#ifndef AUDIO_ENGINE_H
#define AUDIO_ENGINE_H

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

#include "Settings.h"
#include "State.h"
#include "Diffuser.h"
#include "GranularBuffer.h"
#include "RealtimeGranulator.h"
#include "StereoInputSummer.h"
#include "RampSvf.h"

// DaisySP Modules (with USE_DAISYSP_LGPL=1 for Compressor)
#include "daisysp.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

// --- Reverb geometry ---
// Pre-diffuser lengths (samples, primes) and cloud allpass delays (s). Cloud stages 0..3 run in
// series ahead of the tank, 4..5 inside its feedback loop.
constexpr size_t kDiffuserLenL[4] = {149, 211, 331, 461};
constexpr size_t kDiffuserLenR[4] = {157, 223, 397, 467};
constexpr float kCloudDelayL[6] = {0.0131f, 0.0239f, 0.0373f, 0.0491f, 0.0593f, 0.0677f};
constexpr float kCloudDelayR[6] = {0.0147f, 0.0223f, 0.0389f, 0.0541f, 0.0619f, 0.0701f};

// Cloud allpass loop length at SAMPLE_RATE (same rounding as CloudAllpass::Init)
constexpr size_t CloudAllpassLength(float delay_s) {
  return static_cast<size_t>(delay_s * SAMPLE_RATE + 0.5f);
}

constexpr size_t AllpassPoolSize() {
  size_t total = 0;
  for (int i = 0; i < 4; i++) total += kDiffuserLenL[i] + kDiffuserLenR[i];
  for (int i = 0; i < 6; i++) total += CloudAllpassLength(kCloudDelayL[i]) + CloudAllpassLength(kCloudDelayR[i]);
  return total;
}

// Pre-computed per-block parameters to avoid recalculation in the sample loop
struct BlockParams {
  float filter_vca_l[8] = {0.0f};
  float filter_vca_r[8] = {0.0f};
  float pan_factor_l = 0.5f;
  float pan_factor_r = 0.5f;
  float filterRange = 1.0f;
  float verbMix = 0.0f;
  float dryWetMix = 0.5f;
  float oneMinusDryWetMix = 0.5f;
  SignalFlowMode flowMode = SignalFlowMode::GRAN_TO_VERB;
};

/**
 * AudioEngine
 * Complete Effects Pipeline & Audio Engine for Estuary Landscape Granulator.
 * Integrates GranularBuffer, RealtimeGranulator, 8-stage Chamberlin SVF filter bank,
 * 4 Schroeder diffusers + 6 Cloud allpass stages, stereo tape-warble reverb tank,
 * master bus compressor, and output ceiling limiter.
 */
class AudioEngine {
public:
#if defined(__GNUC__) && !defined(__clang__)
  __attribute__((optimize("Os")))
#endif
  AudioEngine() {}
  ~AudioEngine() = default;

  // Granular Engine Primitives
  landscape::GranularBuffer granularBuffer;
  landscape::RealtimeGranulator realtimeGranulator;

  // Effects & Modulators
  daisysp::RampSvf filters[16];
  daisysp::SmoothRandomGenerator arpLfo1;
  daisysp::SmoothRandomGenerator arpLfo2;
  daisysp::Compressor masterComp;

  float current_filter_freqs[16] = {1000.0f};
  float base_filter_freqs[16] = {1000.0f};
  float base_filter_min[16] = {120.0f};
  float base_filter_max[16] = {5500.0f};
  float lfo_outputs[16] = {0.0f};
  float panLfo_val = 0.5f;
  float arpLfo1_val = 0.0f;
  float arpLfo2_val = 0.0f;

  // Polyrhythmic LFO rate multipliers (Just Intonation harmonic ratios)
  static constexpr float LFO_MULTIPLIERS[16] = {
      0.25f,  0.333f, 0.5f,   0.666f, 0.75f,  1.25f,  1.5f,   2.0f,  // VCAs 0..7
      0.125f, 0.375f, 0.625f, 0.833f, 1.125f, 1.333f, 1.666f, 2.5f   // Cutoffs 8..15
  };

#if defined(__GNUC__) && !defined(__clang__)
  __attribute__((optimize("Os")))
#endif
  void Init(float sample_rate = 48000.0f) {
    sr_ = sample_rate;
    const float block_period = static_cast<float>(BLOCK_SIZE) / sample_rate;
    const float lfo_rate = sample_rate / static_cast<float>(BLOCK_SIZE);
    bypass_step_ = 1.0f / (sample_rate * BYPASS_CROSSFADE_MS * 0.001f);
    fb_slew_per_sample_ = (FREEZE_OVERDUB_FEEDBACK - RECORD_FEEDBACK) /
                          (sample_rate * OVERDUB_FEEDBACK_SLEW_MS * 0.001f);
    record_fb_ = RECORD_FEEDBACK;
    record_fb_step_ = 0.0f;

    // 1. Initialize Granular Engine & Smart Input Summer
    input_summer_.Init(sample_rate);
    granularBuffer.Init(sample_rate);
    realtimeGranulator.Init(sample_rate, &granularBuffer);

    // 2. Initialize Master Compressor
    masterComp.Init(sample_rate);
    masterComp.AutoMakeup(false);
    masterComp.SetThreshold(MASTER_COMP_THRESHOLD_DB);
    masterComp.SetRatio(MASTER_COMP_RATIO);
    masterComp.SetAttack(MASTER_COMP_ATTACK_S);
    masterComp.SetRelease(MASTER_COMP_RELEASE_S);
    for (int i = 0; i < 4800; i++) {
      masterComp.Process(0.0f);
    }

    // 3. Initialize Chamberlin SVF Filter Bank
    // Place the 4 stereo bands at the geometric centers (f + 0.5) / 4 of equal log sub-bands
    // so Band 0 and Band 3 sweep symmetrically on both halves of their LFO cycles.
    for (int i = 0; i < 16; i++) {
      filters[i].Init(sample_rate);
      current_filter_freqs[i] = 1000.0f;
      int f = (i < 8) ? (i / 2) : ((i - 8) / 2);
      base_filter_freqs[i] = FILTER_MIN_FREQ *
          std::pow(FILTER_MAX_FREQ / FILTER_MIN_FREQ, (static_cast<float>(f) + 0.5f) / 4.0f);
    }

    for (int i = 0; i < 16; i++) {
      float center = base_filter_freqs[i];
      int start_idx = (i < 8) ? 0 : 8;
      int f = (i - start_idx) / 2;

      if (f == 0) {
        base_filter_min[i] = FILTER_MIN_FREQ;
      } else {
        base_filter_min[i] = std::sqrt(base_filter_freqs[i - 2] * center);
      }

      if (f == 3) {
        base_filter_max[i] = FILTER_MAX_FREQ;
      } else {
        base_filter_max[i] = std::sqrt(center * base_filter_freqs[i + 2]);
      }
    }

    log_freq_min_ = daisysp::fastlog10f(FILTER_MIN_FREQ);
    log_freq_max_ = daisysp::fastlog10f(FILTER_MAX_FREQ);
    inv_log_freq_range_ = 1.0f / (log_freq_max_ - log_freq_min_);

    // 5. Initialize Diffusers & Reverb Tank (all allpass buffers live in one pool)
    initBuffers(sample_rate);

    // 6. Initialize Tape Warble LFOs (+/- TAPE_WARBLE_DEPTH samples)
    tape_lfo_l_ = {0.0f, TAPE_WARBLE_RATE * block_period};
    tape_lfo_r_ = {PhaseFraction(TAPE_WARBLE_RATE * 1.13f), TAPE_WARBLE_RATE * 1.13f * block_period};

    // 7. Initialize 16 Polyrhythmic LFOs + Drift LFOs
    constexpr float base_freq = DEFAULT_BPM / 60.0f;
    for (int i = 0; i < 16; i++) {
      const float lfo_nominal = 1.0f * LFO_MULTIPLIERS[i] * LFO_RATE_MULTIPLIER;
      const float drift_nominal = 0.02f + (i * 0.25f);
      const float churn_s = 0.15f * static_cast<float>(i);

      lfo_center_inc_[i] = base_freq * lfo_nominal * FILTER_LFO_RATE_SCALE * block_period;
      lfos_[i] = {PhaseFraction(churn_s * lfo_nominal), lfo_center_inc_[i]};
      drift_lfos_[i] = {PhaseFraction(churn_s * drift_nominal),
                        drift_nominal * FILTER_LFO_RATE_SCALE * block_period};
    }

    arpLfo1.Init(lfo_rate);
    arpLfo2.Init(lfo_rate);
    arpLfo1.SetFreq((base_freq * 4.0f) * LFO_TUNED_RATE_SCALE);
    arpLfo2.SetFreq((base_freq * 3.0f) * LFO_TUNED_RATE_SCALE);
    pan_lfo_ = {0.0f, (base_freq / 0.50f) * LFO_RATE_MULTIPLIER * LFO_TUNED_RATE_SCALE * block_period};

    cur_tank_delay_l_ = sample_rate * TANK_DELAY_L_S;
    cur_tank_delay_r_ = sample_rate * TANK_DELAY_R_S;
    delay_initialized_ = false;
    params_initialized_ = false;
    effects_ramping_initialized_ = false;
    cloudiness_smoothed = 0.0f;
    ap_mod_phase = 0.0f;
    tank_lpf_l = tank_lpf_r = 0.0f;
    tank_dc_x_l_ = tank_dc_y_l_ = 0.0f;
    tank_dc_x_r_ = tank_dc_y_r_ = 0.0f;
    bank_level_gain_ = bank_level_target_ = 1.0f;
    bank_e_pre_ = bank_e_post_ = 0.0f;
    bypass_mix_ = 0.0f;
  }

  void UpdateParameters(const GranulatorState &state) {
    // Parameter smoothing runs strictly at block rate inside ProcessChunk()
    // so main-loop calls never race with the audio interrupt.
    (void)state;
  }

  /**
   * @brief Pure chunked audio block streaming callback.
   * Chunks large blocks (> BLOCK_SIZE) safely into BLOCK_SIZE = 48 buffers.
   */
  void ProcessBlock(const float *const *in, float *const *out, size_t size, const GranulatorState &state) {
    if (!in || !out || size == 0) {
      return;
    }

    size_t processed = 0;
    while (processed < size) {
      size_t chunk_size = (size - processed > BLOCK_SIZE) ? BLOCK_SIZE : (size - processed);
      const float *in_chunk[2] = { in[0] + processed, in[1] + processed };
      float *out_chunk[2] = { out[0] + processed, out[1] + processed };

      ProcessChunk(in_chunk, out_chunk, chunk_size, state);
      processed += chunk_size;
    }
  }

  // --- Click-Free 10 ms Bypass Crossfade ---
  inline void ProcessBypassCrossfade(float raw_l, float raw_r, float mix_l, float mix_r,
                                     float &out_l, float &out_r, bool bypass_target_state) {
    const float bypass_target = bypass_target_state ? 1.0f : 0.0f;
    if (bypass_mix_ < bypass_target) {
      bypass_mix_ = std::min(bypass_target, bypass_mix_ + bypass_step_);
    } else if (bypass_mix_ > bypass_target) {
      bypass_mix_ = std::max(bypass_target, bypass_mix_ - bypass_step_);
    }
    float fx = 1.0f - bypass_mix_;
    out_l = mix_l * fx + raw_l * bypass_mix_;
    out_r = mix_r * fx + raw_r * bypass_mix_;
  }

  inline void ProcessBypassCrossfade(float raw_in, float mix_l, float mix_r,
                                     float &out_l, float &out_r, bool bypass_target_state) {
    ProcessBypassCrossfade(raw_in, raw_in, mix_l, mix_r, out_l, out_r, bypass_target_state);
  }

  static inline float FastSoftSat(float x) {
    return x / (1.0f + 0.5f * std::fabs(x));
  }

  // Transparent up to the knee, then knee + range * t / (1 + t) with t = (|x| - knee) / range,
  // which approaches 1.0 asymptotically (written with a single division)
  static inline float OutputCeiling(float x) {
    const float a = std::fabs(x);
    if (!(a > OUTPUT_LIMIT_KNEE)) {
      return x;
    }
    constexpr float range = 1.0f - OUTPUT_LIMIT_KNEE;
    const float u = a - OUTPUT_LIMIT_KNEE;
    return std::copysign(OUTPUT_LIMIT_KNEE + range * u / (range + u), x);
  }

  inline float GetPanLfo() const { return panLfo_val; }
  inline float GetStereoInputMix() const { return input_summer_.StereoMix(); }
  inline int GetActiveVoiceCount() const { return realtimeGranulator.GetActiveVoiceCount(); }
  inline float GetBankLevelGain() const { return bank_level_gain_; }
  inline float GetBypassMix() const { return bypass_mix_; }

private:
  /**
   * Reverb tank delay line: a power-of-two ring whose write head moves backwards (the
   * newest sample sits just above it), so indices wrap with a mask.
   */
  struct TankDelay {
    static constexpr size_t kSize = TANK_DELAY_MAX_SAMPLES;
    static constexpr size_t kMask = kSize - 1;
    static_assert((kSize & kMask) == 0, "TANK_DELAY_MAX_SAMPLES must be a power of two");

    float line[kSize];
    size_t write = 0;

    void Reset() {
      std::memset(line, 0, sizeof(line));
      write = 0;
    }

    /**
     * Hermite-reads n samples for a block during which the write head moves one sample per
     * output sample, with the delay ramping by `step` (sample s reads delay + (s + 1) * step).
     * Unless this block's four-tap window straddles the end of the ring (~1% of blocks), the
     * taps are read without masking.
     */
    void ReadBlock(float *out, size_t n, float delay, float step) const {
      const float d_end = delay + step * static_cast<float>(n);
      const int32_t head = static_cast<int32_t>(write);
      const int32_t lo = head - static_cast<int32_t>(n) + static_cast<int32_t>(std::fmin(delay, d_end) - 1.0f) - 1;
      const int32_t hi = head + static_cast<int32_t>(std::fmax(delay, d_end) + 1.0f) + 2;
      constexpr int32_t kSizeI = static_cast<int32_t>(kSize);
      if (lo >= 0 && hi < kSizeI) {
        ReadRun<false>(out, n, delay, step, head);
      } else if (lo >= kSizeI && hi < 2 * kSizeI) {
        ReadRun<false>(out, n, delay, step, head - kSizeI);
      } else {
        ReadRun<true>(out, n, delay, step, head);
      }
    }

  private:
    template <bool kWrap>
    inline void ReadRun(float *out, size_t n, float delay, float step, int32_t head) const {
      for (size_t s = 0; s < n; s++) {
        delay += step;
        const int32_t di = static_cast<int32_t>(delay);
        const float f = delay - static_cast<float>(di);
        const int32_t t = head - static_cast<int32_t>(s) + di;
        float xm1, x0, x1, x2;
        if (kWrap) {
          xm1 = line[(t - 1) & kMask];
          x0 = line[t & kMask];
          x1 = line[(t + 1) & kMask];
          x2 = line[(t + 2) & kMask];
        } else {
          const float *q = line + t;
          xm1 = q[-1];
          x0 = q[0];
          x1 = q[1];
          x2 = q[2];
        }
        const float c = (x1 - xm1) * 0.5f;
        const float v = x0 - x1;
        const float w = c + v;
        const float a = w + v + (x2 - x0) * 0.5f;
        const float b_neg = w + a;
        out[s] = (((a * f) - b_neg) * f + c) * f + x0;
      }
    }
  };

  void initBuffers(float sample_rate) {
    tank_l_.Reset();
    tank_r_.Reset();
    float *pool = allpass_pool_;
    for (int i = 0; i < 4; i++) {
      pre_diff_l_[i].Init(pool, kDiffuserLenL[i], 0.60f);
      pool += kDiffuserLenL[i];
      pre_diff_r_[i].Init(pool, kDiffuserLenR[i], 0.60f);
      pool += kDiffuserLenR[i];
    }
    for (int i = 0; i < 6; i++) {
      const size_t len_l = CloudAllpassLength(kCloudDelayL[i]);
      cloud_l_[i].Init(pool, len_l, kCloudDelayL[i], sample_rate);
      pool += len_l;
      const size_t len_r = CloudAllpassLength(kCloudDelayR[i]);
      cloud_r_[i].Init(pool, len_r, kCloudDelayR[i], sample_rate);
      pool += len_r;
    }
  }

  static float PhaseFraction(float cycles) {
    return cycles - std::floor(cycles);
  }

  // sin(2 pi * cycles), cycles in [0, 1): folded to [-pi/2, pi/2] and an odd 7th-order fit
  // (|error| < 1e-6, |output| < 1). Block-rate LFOs only; ~10x cheaper than sinf().
  static inline float SinCycles(float cycles) {
    float x = cycles - 0.5f; // sin(2 pi c) = -sin(2 pi x), x in [-0.5, 0.5)
    if (x > 0.25f) {
      x = 0.5f - x;
    } else if (x < -0.25f) {
      x = -0.5f - x;
    }
    const float t = -TWOPI_F * x;
    const float t2 = t * t;
    return t * (0.99999660f + t2 * (-0.16664828f + t2 * (0.0083063273f + t2 * -0.00018363701f)));
  }

  // Block-rate sine LFO; phase and increment in cycles
  struct SineLfo {
    float phase = 0.0f;
    float inc = 0.0f;
    inline float Process() {
      const float out = SinCycles(phase);
      phase += inc;
      if (phase >= 1.0f) {
        phase -= 1.0f;
      }
      return out;
    }
  };

  void processLfos() {
    for (int i = 0; i < 16; i++) {
      // Each LFO's rate drifts +/-25% around its center
      lfos_[i].inc = lfo_center_inc_[i] * (1.0f + (drift_lfos_[i].Process() * 0.25f));
      lfo_outputs[i] = (lfos_[i].Process() + 1.0f) * 0.5f;
    }
    panLfo_val = (pan_lfo_.Process() + 1.0f) * 0.5f;
    arpLfo1_val = (arpLfo1.Process() + 1.0f) * 0.5f;
    arpLfo2_val = (arpLfo2.Process() + 1.0f) * 0.5f;
  }

  void prepareBlockParams(BlockParams &p, const GranulatorState &s) {
    p.filterRange = sm_filterRange_;
    p.verbMix = sm_verbMix_;
    p.dryWetMix = sm_dryWetMix_;
    p.oneMinusDryWetMix = 1.0f - sm_dryWetMix_;
    p.flowMode = s.flowMode;

    constexpr int kNumActiveVcaChannels = ENABLE_POST_FILTER_BANK ? 8 : 4;
    for (int i = 0; i < kNumActiveVcaChannels; i++) {
      p.filter_vca_l[i] = FILTER_VCA_FLOOR + (1.0f - FILTER_VCA_FLOOR) * lfo_outputs[i * 2];
      p.filter_vca_r[i] = FILTER_VCA_FLOOR + (1.0f - FILTER_VCA_FLOOR) * lfo_outputs[i * 2 + 1];
    }

    p.pan_factor_l = 0.18f + (panLfo_val * 0.82f);
    p.pan_factor_r = 0.18f + ((1.0f - panLfo_val) * 0.82f);
  }

  void updateFilters(const GranulatorState &state) {
    if (!ENABLE_POST_FILTER_BANK && state.filterRange <= FILTER_BANK_MIN_RANGE &&
        sm_filterRange_ <= FILTER_BANK_MIN_RANGE) {
      return;
    }

    float filter_push = state.filterRange * state.filterRange * state.filterRange;
    float dynamic_max_res = FILTER_MAX_RESONANCE + (filter_push * 0.03f);
    float dynamic_lfo_amount = FILTER_RES_LFO_AMOUNT + (filter_push * 0.03f);
    float res_range = dynamic_max_res - FILTER_MIN_RESONANCE;
    float mapped_verb_res = FILTER_MIN_RESONANCE + (state.filterRange * res_range);

    for (int i = 0; i < NUM_RUNNING_FILTERS; i++) {
      float lfo = lfo_outputs[(i % 8) + 8];
      float bipolar_lfo = (lfo * 2.0f) - 1.0f;

      float center = base_filter_freqs[i];
      float min_freq = base_filter_min[i];
      float max_freq = base_filter_max[i];

      float target;
      if (bipolar_lfo > 0.0f) {
        target = center + bipolar_lfo * (max_freq - center);
      } else {
        target = center + bipolar_lfo * (center - min_freq);
      }

      current_filter_freqs[i] += (target - current_filter_freqs[i]) * 0.1f;

      float base_offset = (i % 2) * 0.02f;
      float res_lfo_mod = lfo * dynamic_lfo_amount;
      float base_res = mapped_verb_res + base_offset + res_lfo_mod;

      float log_freq = daisysp::fastlog10f(std::max(current_filter_freqs[i], FILTER_MIN_FREQ));
      float norm_log_freq = std::min(1.0f, std::max(0.0f, (log_freq - log_freq_min_) * inv_log_freq_range_));
      float res = std::min(FILTER_RES_CEILING,
                           std::max(0.0f, base_res - norm_log_freq * HIGH_FREQ_DAMPING));

      filters[i].SetTarget(current_filter_freqs[i], res);
    }
  }

  void processFilterBank(const float *in_l, const float *in_r, float *out_l,
                         float *out_r, const BlockParams &p, size_t num_samples,
                         int start_filter_idx, int ch_offset) {
    // Bands 0..1 are lowpass (120..812 Hz); bands 2..3 are bandpass (812..5500 Hz)
    // and receive ~2x bandpass makeup gain without attenuating the lowpass fundamental bands.
    static constexpr float LOG_GAINS[8] = {
        0.26000000f, 0.28000000f, 0.42000000f, 0.36000000f,
        0.41019184f, 0.55208764f, 0.74306602f, 1.00000000f};
    constexpr float kPreBankMakeup =
        1.0f / ((0.26000000f + 0.28000000f + 0.20000000f + 0.16000000f) * 0.55f);
    constexpr float kPostBankMakeup =
        1.0f / ((0.41019184f + 0.55208764f + 0.74306602f + 1.00000000f) * 0.55f);
    const float makeup_gain = (ch_offset == 0) ? kPreBankMakeup : kPostBankMakeup;
    const float inv_samples = 1.0f / static_cast<float>(num_samples);

    processBankChannel(in_l, out_l, p.filter_vca_l, prev_filter_gain_l_,
                       start_filter_idx, ch_offset, makeup_gain, LOG_GAINS,
                       num_samples, inv_samples);
    processBankChannel(in_r, out_r, p.filter_vca_r, prev_filter_gain_r_,
                       start_filter_idx + 1, ch_offset, makeup_gain, LOG_GAINS,
                       num_samples, inv_samples);
  }

  void processBankChannel(const float *in, float *out, const float *vca,
                          float *prev_gain, int first_filter, int ch_offset,
                          float makeup_gain, const float *log_gains,
                          size_t num_samples, float inv_samples) {
    float gain[4], step[4];
    for (int sub_ch = 0; sub_ch < 4; sub_ch++) {
      const int ch_idx = ch_offset + sub_ch;
      const float target_gain = vca[ch_idx] * log_gains[ch_idx] * makeup_gain;
      gain[sub_ch] = effects_ramping_initialized_ ? prev_gain[ch_idx] : target_gain;
      step[sub_ch] = (target_gain - gain[sub_ch]) * inv_samples;
      prev_gain[ch_idx] = target_gain;
      filters[first_filter + 2 * sub_ch].BeginBlock(inv_samples);
    }
    daisysp::RampSvf f0 = filters[first_filter], f1 = filters[first_filter + 2];
    daisysp::RampSvf f2 = filters[first_filter + 4], f3 = filters[first_filter + 6];
    float g0 = gain[0], g1 = gain[1], g2 = gain[2], g3 = gain[3];
    const float s0 = step[0], s1 = step[1], s2 = step[2], s3 = step[3];
    for (size_t s = 0; s < num_samples; s++) {
      const float x = in[s];
      g0 += s0;
      float y = f0.ProcessLow(x) * g0;
      g1 += s1;
      y += f1.ProcessLow(x) * g1;
      g2 += s2;
      y += f2.ProcessBand(x) * g2;
      g3 += s3;
      y += f3.ProcessBand(x) * g3;
      out[s] = y;
    }
    f0.EndBlock();
    f1.EndBlock();
    f2.EndBlock();
    f3.EndBlock();
    filters[first_filter] = f0;
    filters[first_filter + 2] = f1;
    filters[first_filter + 4] = f2;
    filters[first_filter + 6] = f3;
  }

  void processEffectsChunk(float *gr_out_l, float *gr_out_r, const float *live,
                           float *wet_l, float *wet_r, const BlockParams &p,
                           size_t n) {
    float push_factor = cloudiness_smoothed * cloudiness_smoothed * cloudiness_smoothed;
    float tank_feedback = cloudiness_smoothed * DIFFUSION_TANK_MAX_FEEDBACK;

    // --- 1. PRE-PROCESSING & PANNING ---
    float *eff_in_l = eff_in_l_;
    float *eff_in_r = eff_in_r_;

    // Constant-power panLfo normalization (pan_gain_l^2 + pan_gain_r^2 = 0.75^2 = 9/16)
    // eliminates the +1.71 dB power swell at the LFO extremes and cancels RealtimeGranulator's 1/9 trim.
    float pan_sq = p.pan_factor_l * p.pan_factor_l + p.pan_factor_r * p.pan_factor_r;
    float pan_norm = (pan_sq > 1.0e-6f) ? (0.75f / std::sqrt(pan_sq)) : 0.53033009f;
    float pan_gain_l = p.pan_factor_l * pan_norm;
    float pan_gain_r = p.pan_factor_r * pan_norm;

    const float inv_n = 1.0f / static_cast<float>(n);
    const float start_filter_range = effects_ramping_initialized_ ? prev_filter_range_ : p.filterRange;
    const float step_filter_range = (p.filterRange - start_filter_range) * inv_n;
    const float start_verb_mix = effects_ramping_initialized_ ? prev_verb_mix_ : p.verbMix;
    const float start_pan_l = effects_ramping_initialized_ ? prev_pan_gain_l_ : pan_gain_l;
    const float start_pan_r = effects_ramping_initialized_ ? prev_pan_gain_r_ : pan_gain_r;
    const float step_pan_l = (pan_gain_l - start_pan_l) * inv_n;
    const float step_pan_r = (pan_gain_r - start_pan_r) * inv_n;
    float cur_pan_l = start_pan_l;
    float cur_pan_r = start_pan_r;

    // Pan grain outputs once in-place (reused directly in Step 5 dry grain mix)
    if (p.flowMode == SignalFlowMode::GRAN_TO_VERB) {
      for (size_t s = 0; s < n; s++) {
        cur_pan_l += step_pan_l;
        cur_pan_r += step_pan_r;
        const float panned_l = gr_out_l[s] * cur_pan_l;
        const float panned_r = gr_out_r[s] * cur_pan_r;
        gr_out_l[s] = panned_l;
        gr_out_r[s] = panned_r;
        eff_in_l[s] = panned_l;
        eff_in_r[s] = panned_r;
      }
    } else {
      for (size_t s = 0; s < n; s++) {
        cur_pan_l += step_pan_l;
        cur_pan_r += step_pan_r;
        gr_out_l[s] *= cur_pan_l;
        gr_out_r[s] *= cur_pan_r;
        eff_in_l[s] = live[s];
        eff_in_r[s] = live[s];
      }
    }

    // --- 2. PRE-REVERB FILTER BANK ---
    if (ENABLE_PRE_FILTER_BANK) {
      if (p.filterRange > FILTER_BANK_MIN_RANGE || start_filter_range > FILTER_BANK_MIN_RANGE) {
        float *filtered_pre_l = filtered_l_;
        float *filtered_pre_r = filtered_r_;
        processFilterBank(eff_in_l, eff_in_r, filtered_pre_l, filtered_pre_r, p, n, 0, 0);

        float fr = start_filter_range;
        float g = bank_level_gain_;
        const float g_step = (bank_level_target_ - g) * inv_n;
        float e_pre = 0.0f, e_post = 0.0f;
        for (size_t s = 0; s < n; s++) {
          fr += step_filter_range;
          g += g_step;
          const float in_l = eff_in_l[s], in_r = eff_in_r[s];
          const float out_l = in_l + (filtered_pre_l[s] - in_l) * fr;
          const float out_r = in_r + (filtered_pre_r[s] - in_r) * fr;
          e_pre += in_l * in_l + in_r * in_r;
          e_post += out_l * out_l + out_r * out_r;
          const float blend = std::min(1.0f, std::max(0.0f, (fr - FILTER_BANK_MIN_RANGE) * 20.0f));
          const float eff_g = 1.0f + (g - 1.0f) * blend;
          eff_in_l[s] = out_l * eff_g;
          eff_in_r[s] = out_r * eff_g;
        }
        bank_level_gain_ = bank_level_target_;
        const float k = kBankLevelSmooth * static_cast<float>(n);
        if (e_pre > kBankLevelFloor * static_cast<float>(n)) {
          bank_e_pre_ += k * (e_pre - bank_e_pre_);
          bank_e_post_ += k * (e_post - bank_e_post_);
          bank_level_target_ = std::min(kBankLevelMaxGain,
                                        std::max(kBankLevelMinGain,
                                                 std::sqrt(bank_e_pre_ / (bank_e_post_ + 1e-20f))));
        } else {
          // Decay both energy accumulators together during silence so their ratio
          // (and bank_level_target_) holds steady without a step jump on the next attack.
          bank_e_pre_ *= (1.0f - k);
          bank_e_post_ *= (1.0f - k);
        }
      } else {
        // blend already faded eff_g to 1.0 as filterRange approached FILTER_BANK_MIN_RANGE
        bank_level_gain_ = 1.0f;
        bank_level_target_ = 1.0f;
      }

      // In GRAN_TO_VERB, route the filtered & level-compensated granular cloud to the dry-grain
      // side of the VerbMix crossfade too, so Knob 4 (Filter/Diffusion) works at every VerbMix.
      if (p.flowMode == SignalFlowMode::GRAN_TO_VERB) {
        std::memcpy(gr_out_l, eff_in_l, n * sizeof(float));
        std::memcpy(gr_out_r, eff_in_r, n * sizeof(float));
      }
    }

    // --- 2.5 PRE-DIFFUSION ALLPASS FILTER BANK ---
    if (ENABLE_ALLPASS_FILTER_BANK) {
      AllpassStage::ProcessChain<4>(pre_diff_l_, eff_in_l, n);
      AllpassStage::ProcessChain<4>(pre_diff_r_, eff_in_r, n);
    }

    // --- 3. CLOUD PROCESSING & REVERB TANK ---
    // Stages 0..3 are purely in series ahead of the tank feedback loop.
    AllpassStage::ProcessChain<4>(cloud_l_, eff_in_l, n);
    AllpassStage::ProcessChain<4>(cloud_r_, eff_in_r, n);

    float dynamic_cross_fb = REVERB_CROSS_FEEDBACK + (push_factor * 0.10f);
    const float cross_cos = std::sqrt(std::max(0.0f, 1.0f - dynamic_cross_fb * dynamic_cross_fb));
    float injection_scale = 0.35f * (1.0f - tank_feedback * 0.45f);
    float damp_factor = 0.55f - (cloudiness_smoothed * 0.20f);

    float verb_makeup = std::sqrt(std::max(0.0f, 1.0f - tank_feedback * tank_feedback)) / injection_scale;
    float target_raw_wet_gain = verb_makeup;

    float target_delay_l = (sr_ * TANK_DELAY_L_S) + tape_lfo_l_.Process() * TAPE_WARBLE_DEPTH;
    float target_delay_r = (sr_ * TANK_DELAY_R_S) + tape_lfo_r_.Process() * TAPE_WARBLE_DEPTH;
    if (!delay_initialized_) {
      delay_initialized_ = true;
      cur_tank_delay_l_ = target_delay_l;
      cur_tank_delay_r_ = target_delay_r;
    }
    const float step_delay_l = (target_delay_l - cur_tank_delay_l_) * inv_n;
    const float step_delay_r = (target_delay_r - cur_tank_delay_r_) * inv_n;

    float cur_tank_fb = effects_ramping_initialized_ ? prev_tank_fb_ : tank_feedback;
    float cur_raw_wet_gain = effects_ramping_initialized_ ? prev_raw_wet_gain_ : target_raw_wet_gain;
    float cur_injection_scale = effects_ramping_initialized_ ? prev_injection_scale_ : injection_scale;
    float cur_damp_factor = effects_ramping_initialized_ ? prev_damp_factor_ : damp_factor;
    float cur_cross_fb = effects_ramping_initialized_ ? prev_cross_fb_ : dynamic_cross_fb;
    float cur_cross_cos = effects_ramping_initialized_ ? prev_cross_cos_ : cross_cos;

    const float step_tank_fb = (tank_feedback - cur_tank_fb) * inv_n;
    const float step_raw_wet_gain = (target_raw_wet_gain - cur_raw_wet_gain) * inv_n;
    const float step_injection_scale = (injection_scale - cur_injection_scale) * inv_n;
    const float step_damp_factor = (damp_factor - cur_damp_factor) * inv_n;
    const float step_cross_fb = (dynamic_cross_fb - cur_cross_fb) * inv_n;
    const float step_cross_cos = (cross_cos - cur_cross_cos) * inv_n;

    // The tank delay (>= 2208 samples) is far longer than a block, so no read in this block
    // sees a sample written in it: the tank runs as block stages A..D, one after another.
    float *raw_wet_l = raw_wet_l_;
    float *raw_wet_r = raw_wet_r_;
    float *tank_in_l = filtered_l_;
    float *tank_in_r = filtered_r_;

    // A. Tape-warbled Hermite reads of the tank outputs (into raw_wet, scaled in stage D)
    tank_l_.ReadBlock(raw_wet_l, n, cur_tank_delay_l_, step_delay_l);
    tank_r_.ReadBlock(raw_wet_r, n, cur_tank_delay_r_, step_delay_r);

    // B. Orthonormal rotation cross-feedback, soft-saturated injection, lowpass damping
    {
      float fb = cur_tank_fb, inj = cur_injection_scale, damp = cur_damp_factor;
      float cfb = cur_cross_fb, ccos = cur_cross_cos;
      float lpf_l = tank_lpf_l, lpf_r = tank_lpf_r;
      for (size_t s = 0; s < n; s++) {
        fb += step_tank_fb;
        inj += step_injection_scale;
        damp += step_damp_factor;
        cfb += step_cross_fb;
        ccos += step_cross_cos;
        const float tank_out_l = raw_wet_l[s];
        const float tank_out_r = raw_wet_r[s];
        const float feed_l = ccos * tank_out_l - cfb * tank_out_r;
        const float feed_r = cfb * tank_out_l + ccos * tank_out_r;
        const float verb_in_l = FastSoftSat(eff_in_l[s] * inj);
        const float verb_in_r = FastSoftSat(eff_in_r[s] * inj);
        const float x_l = verb_in_l + (feed_l * fb);
        const float x_r = verb_in_r + (feed_r * fb);
        lpf_l += damp * (x_l - lpf_l);
        lpf_r += damp * (x_r - lpf_r);
        tank_in_l[s] = lpf_l;
        tank_in_r[s] = lpf_r;
      }
      tank_lpf_l = landscape::FlushDenormal(lpf_l);
      tank_lpf_r = landscape::FlushDenormal(lpf_r);
    }

    // C. In-tank allpasses (stages 4 and 5)
    AllpassStage::ProcessChain<2>(cloud_l_ + 4, tank_in_l, n);
    AllpassStage::ProcessChain<2>(cloud_r_ + 4, tank_in_r, n);

    // D. Subsonic DC blocker (5.3 Hz) inside the loop, tank write, post-tank makeup gain
    {
      float x_l = tank_dc_x_l_, y_l = tank_dc_y_l_, x_r = tank_dc_x_r_, y_r = tank_dc_y_r_;
      float wet_gain = cur_raw_wet_gain;
      size_t head_l = tank_l_.write, head_r = tank_r_.write;
      for (size_t s = 0; s < n; s++) {
        wet_gain += step_raw_wet_gain;
        const float in_l = tank_in_l[s];
        const float in_r = tank_in_r[s];
        y_l = in_l - x_l + 0.9993f * y_l;
        x_l = in_l;
        y_r = in_r - x_r + 0.9993f * y_r;
        x_r = in_r;
        tank_l_.line[head_l] = y_l;
        tank_r_.line[head_r] = y_r;
        head_l = (head_l - 1) & TankDelay::kMask;
        head_r = (head_r - 1) & TankDelay::kMask;
        raw_wet_l[s] *= wet_gain;
        raw_wet_r[s] *= wet_gain;
      }
      tank_l_.write = head_l;
      tank_r_.write = head_r;
      tank_dc_x_l_ = x_l;
      tank_dc_y_l_ = landscape::FlushDenormal(y_l);
      tank_dc_x_r_ = x_r;
      tank_dc_y_r_ = landscape::FlushDenormal(y_r);
    }

    cur_tank_delay_l_ = target_delay_l;
    cur_tank_delay_r_ = target_delay_r;
    prev_tank_fb_ = tank_feedback;
    prev_raw_wet_gain_ = target_raw_wet_gain;
    prev_injection_scale_ = injection_scale;
    prev_damp_factor_ = damp_factor;
    prev_cross_fb_ = dynamic_cross_fb;
    prev_cross_cos_ = cross_cos;

    // --- 4. POST-REVERB FILTER BANK (Optional) ---
    const float *post_wet_l = raw_wet_l;
    const float *post_wet_r = raw_wet_r;
    if (ENABLE_POST_FILTER_BANK) {
      float *filtered_post_l = filtered_l_;
      float *filtered_post_r = filtered_r_;
      processFilterBank(raw_wet_l, raw_wet_r, filtered_post_l, filtered_post_r, p, n, 8, 4);
      float fr = start_filter_range;
      for (size_t s = 0; s < n; s++) {
        fr += step_filter_range;
        eff_in_l_[s] = raw_wet_l[s] + (filtered_post_l[s] - raw_wet_l[s]) * fr;
        eff_in_r_[s] = raw_wet_r[s] + (filtered_post_r[s] - raw_wet_r[s]) * fr;
      }
      post_wet_l = eff_in_l_;
      post_wet_r = eff_in_r_;
    }

    // --- 5. FINAL EQUAL-POWER VERB MIX & STEREO WIDENING ---
    float g_grain = std::sqrt(std::max(0.0f, 1.0f - start_verb_mix));
    float g_verb = std::sqrt(std::max(0.0f, start_verb_mix)) * REVERB_OUTPUT_GAIN;
    const float step_g_grain = (std::sqrt(std::max(0.0f, 1.0f - p.verbMix)) - g_grain) * inv_n;
    const float step_g_verb = (std::sqrt(std::max(0.0f, p.verbMix)) * REVERB_OUTPUT_GAIN - g_verb) * inv_n;

    for (size_t s = 0; s < n; s++) {
      g_grain += step_g_grain;
      g_verb += step_g_verb;

      // 1.5x Mid/Side widening
      const float pwl = post_wet_l[s];
      const float pwr = post_wet_r[s];
      const float mid = (pwl + pwr) * 0.5f;
      const float side = (pwl - pwr) * (0.5f * REVERB_STEREO_WIDTH);

      const float wide_wet_l = mid + side;
      const float wide_wet_r = mid - side;

      wet_l[s] = gr_out_l[s] * g_grain + wide_wet_l * g_verb;
      wet_r[s] = gr_out_r[s] * g_grain + wide_wet_r * g_verb;
    }

    prev_filter_range_ = p.filterRange;
    prev_verb_mix_ = p.verbMix;
    prev_pan_gain_l_ = pan_gain_l;
    prev_pan_gain_r_ = pan_gain_r;
    effects_ramping_initialized_ = true;
  }

  void ProcessChunk(const float *const *in, float *const *out, size_t n, const GranulatorState &state) {
    if (!params_initialized_) {
      sm_dryWetMix_ = state.dryWetMix;
      sm_verbMix_ = state.verbMix;
      sm_filterRange_ = state.filterRange;
      cloudiness_smoothed = std::max(0.0f, std::min(0.99f, state.verbFeedback));
      prev_dry_gain_ = std::sqrt(std::max(0.0f, 1.0f - sm_dryWetMix_));
      prev_wet_gain_ = std::sqrt(std::max(0.0f, sm_dryWetMix_)) * MASTER_OUTPUT_GAIN;
      params_initialized_ = true;
    } else {
      constexpr float kParamSmooth = 0.20f;
      sm_dryWetMix_ += kParamSmooth * (state.dryWetMix - sm_dryWetMix_);
      sm_verbMix_ += kParamSmooth * (state.verbMix - sm_verbMix_);
      sm_filterRange_ += kParamSmooth * (state.filterRange - sm_filterRange_);
      if (std::fabs(state.dryWetMix - sm_dryWetMix_) < 1e-5f) sm_dryWetMix_ = state.dryWetMix;
      if (std::fabs(state.verbMix - sm_verbMix_) < 1e-5f) sm_verbMix_ = state.verbMix;
      if (std::fabs(state.filterRange - sm_filterRange_) < 1e-5f) sm_filterRange_ = state.filterRange;
    }

    // 0. Smart L / Stereo Input Summer:
    // In L alone when In R is unpatched (no -6 dB drop), or smooth crossfade to (L+R)/2 when In R carries signal.
    input_summer_.Prepare(in[1], n);
    input_summer_.ProcessBlock(in[0], in[1], mono_in_, n);

    // 1. Update block modulators (LFOs)
    processLfos();

    // 2. Synchronize Granulator parameters
    landscape::ArpScaleMode gran_arp = landscape::ArpScaleMode::CHROMATIC;
    if (state.arpMode == ArpMode::FIFTHS) {
      gran_arp = landscape::ArpScaleMode::FIFTHS;
    } else if (state.arpMode == ArpMode::CUSTOM) {
      gran_arp = landscape::ArpScaleMode::CUSTOM;
    }
    realtimeGranulator.SetParameters(state.grainSize, state.grainDensity, state.grainArp,
                                     state.randomness, gran_arp, arpLfo1_val, arpLfo2_val);
    realtimeGranulator.SetReverseProbability(
        state.grainDirection == GrainDirection::REVERSE ? 1.0f
        : state.grainDirection == GrainDirection::RANDOM ? RANDOM_DIRECTION_REVERSE_CHANCE
                                                         : 0.0f);

    // 3. Smooth Reverb Feedback
    const float verbFeedback = std::max(0.0f, std::min(0.99f, state.verbFeedback));
    cloudiness_smoothed += (verbFeedback - cloudiness_smoothed) * 0.1f;

    // 4. Update Allpasses & Diffusers
    ap_mod_phase += (0.008f / TWOPI_F) * (static_cast<float>(n) / 16.0f);
    if (ap_mod_phase >= 1.0f) {
      ap_mod_phase -= 1.0f;
    }
    const float ap_cos_phase = (ap_mod_phase + 0.25f >= 1.0f) ? (ap_mod_phase - 0.75f) : (ap_mod_phase + 0.25f);
    float mod_l = SinCycles(ap_mod_phase) * 0.05f;
    float mod_r = SinCycles(ap_cos_phase) * 0.05f;
    float push_factor = cloudiness_smoothed * cloudiness_smoothed * cloudiness_smoothed;
    float base_rt = 0.30f + (cloudiness_smoothed * 0.50f) + (push_factor * 0.15f);
    float rt_outer_l = std::min(0.92f, base_rt + mod_l) * state.diffusionAmount;
    float rt_outer_r = std::min(0.92f, base_rt + mod_r) * state.diffusionAmount;
    // Keep a 35% minimum diffusion floor on the in-tank allpasses (stages 4..5) so the
    // reverb tank never collapses into a bare 2-tap slapback when Knob 4 is low.
    const float tank_diff_scale = 0.35f + 0.65f * state.diffusionAmount;
    float rt_tank_l = std::min(0.92f, base_rt + mod_l) * tank_diff_scale;
    float rt_tank_r = std::min(0.92f, base_rt + mod_r) * tank_diff_scale;

    for (int i = 0; i < 4; i++) {
      cloud_l_[i].SetRevTime(rt_outer_l, n);
      cloud_r_[i].SetRevTime(rt_outer_r, n);
    }
    for (int i = 4; i < 6; i++) {
      cloud_l_[i].SetRevTime(rt_tank_l, n);
      cloud_r_[i].SetRevTime(rt_tank_r, n);
    }
    float diff_gain = std::min(0.70f, 0.60f * state.diffusionAmount);
    for (int i = 0; i < 4; i++) {
      pre_diff_l_[i].SetFeedback(diff_gain, n);
      pre_diff_r_[i].SetFeedback(diff_gain, n);
    }

    // 5. Overdub Feedback Slew
    const bool is_held = state.isFrozen && (state.freezeMode != FreezeMode::OVERDUB);
    const bool is_overdub = state.isFrozen && (state.freezeMode == FreezeMode::OVERDUB);
    const float fb_target = (state.isFrozen && is_overdub) ? FREEZE_OVERDUB_FEEDBACK : RECORD_FEEDBACK;
    const float fb_max = fb_slew_per_sample_ * static_cast<float>(n);
    const float fb_end = record_fb_ + std::max(-fb_max, std::min(fb_max, fb_target - record_fb_));
    record_fb_step_ = (fb_end - record_fb_) / static_cast<float>(n);

    // 6. Write to GranularBuffer
    if (state.flowMode != SignalFlowMode::VERB_TO_GRAN) {
      granularBuffer.WriteBlock(mono_in_, n, record_fb_, record_fb_step_, is_overdub, is_held);
    }

    // 7. Render Synchronous Grains
    realtimeGranulator.ProcessBlockStereo(granularBuffer, granularBlockL_, granularBlockR_, n);

    // 8. Pre-filter and Effects Pipeline
    updateFilters(state);
    BlockParams params;
    prepareBlockParams(params, state);

    processEffectsChunk(granularBlockL_, granularBlockR_, mono_in_,
                        wet_l_chunk_, wet_r_chunk_, params, n);

    // 9. Feedback to GranularBuffer if VERB_TO_GRAN
    if (state.flowMode == SignalFlowMode::VERB_TO_GRAN) {
      float *verb_mono = eff_in_l_;
      for (size_t i = 0; i < n; i++) {
        verb_mono[i] = (wet_l_chunk_[i] + wet_r_chunk_[i]) * 0.5f;
      }
      granularBuffer.WriteBlock(verb_mono, n, record_fb_, record_fb_step_, is_overdub, is_held);
    }

    // 10. Equal-Power Dry/Wet, Master Compressor, Output Ceiling & Bypass Crossfade
    const float target_wet_gain = std::sqrt(params.dryWetMix) * MASTER_OUTPUT_GAIN;
    const float target_dry_gain = std::sqrt(params.oneMinusDryWetMix);
    const float inv_n = 1.0f / static_cast<float>(n);
    const float dry_step = (target_dry_gain - prev_dry_gain_) * inv_n;
    const float wet_step = (target_wet_gain - prev_wet_gain_) * inv_n;
    float cur_dry_gain = prev_dry_gain_;
    float cur_wet_gain = prev_wet_gain_;

    const float stereo_mix = input_summer_.StereoMix();
    const bool bypass_engaged = (!state.bypass && bypass_mix_ == 0.0f);
    for (size_t i = 0; i < n; i++) {
      cur_dry_gain += dry_step;
      cur_wet_gain += wet_step;
      const float raw_l = in[0][i];
      const float raw_r = raw_l + stereo_mix * (in[1][i] - raw_l);
      float mix_l = raw_l * cur_dry_gain + wet_l_chunk_[i] * cur_wet_gain;
      float mix_r = raw_r * cur_dry_gain + wet_r_chunk_[i] * cur_wet_gain;

      // Master Compressor (-3 dBFS threshold, 4:1 ratio)
      masterComp.Process(std::max(std::fabs(mix_l), std::fabs(mix_r)));
      mix_l = masterComp.Apply(mix_l);
      mix_r = masterComp.Apply(mix_r);

      // Output Ceiling Limiter (0.80 knee)
      mix_l = OutputCeiling(mix_l);
      mix_r = OutputCeiling(mix_r);

      if (bypass_engaged) {
        out[0][i] = mix_l;
        out[1][i] = mix_r;
      } else {
        // Click-free 10 ms bypass crossfade
        ProcessBypassCrossfade(raw_l, raw_r, mix_l, mix_r, out[0][i], out[1][i], state.bypass);
      }
    }

    prev_dry_gain_ = target_dry_gain;
    prev_wet_gain_ = target_wet_gain;
    record_fb_ = fb_end;
    record_fb_step_ = 0.0f;
  }

  // --- Internal State ---
  static constexpr int NUM_RUNNING_FILTERS = ENABLE_POST_FILTER_BANK ? 16 : 8;

  SineLfo tape_lfo_l_ = {0.0f, 0.0f};
  SineLfo tape_lfo_r_ = {0.0f, 0.0f};
  SineLfo lfos_[16] = {};
  SineLfo drift_lfos_[16] = {};
  SineLfo pan_lfo_ = {0.0f, 0.0f};
  float lfo_center_inc_[16] = {0.0f};

  float sr_ = 48000.0f;
  float cloudiness_smoothed = 0.0f;
  float ap_mod_phase = 0.0f;
  float tank_lpf_l = 0.0f, tank_lpf_r = 0.0f;
  float tank_dc_x_l_ = 0.0f, tank_dc_y_l_ = 0.0f;
  float tank_dc_x_r_ = 0.0f, tank_dc_y_r_ = 0.0f;

  float log_freq_min_ = 0.0f;
  float log_freq_max_ = 0.0f;
  float inv_log_freq_range_ = 1.0f;

  float bank_level_gain_ = 1.0f, bank_level_target_ = 1.0f;
  float bank_e_pre_ = 0.0f, bank_e_post_ = 0.0f;
  static constexpr float kBankLevelSmooth = 1.0f / (0.25f * 48000.0f);
  static constexpr float kBankLevelFloor = 1.0e-8f;
  static constexpr float kBankLevelMinGain = 0.5f;
  static constexpr float kBankLevelMaxGain = 2.4f;

  float bypass_mix_ = 0.0f;
  float bypass_step_ = 0.0f;

  float record_fb_ = RECORD_FEEDBACK;
  float record_fb_step_ = 0.0f;
  float fb_slew_per_sample_ = 0.0f;

  bool params_initialized_ = false;
  float sm_dryWetMix_ = 0.5f;
  float sm_verbMix_ = 0.0f;
  float sm_filterRange_ = 1.0f;
  float prev_dry_gain_ = 0.5f;
  float prev_wet_gain_ = 1.0f;

  bool delay_initialized_ = false;
  float cur_tank_delay_l_ = 2256.0f, cur_tank_delay_r_ = 3504.0f;
  float prev_tank_fb_ = 0.0f;
  float prev_injection_scale_ = 0.35f;
  float prev_damp_factor_ = 0.55f;
  float prev_cross_fb_ = REVERB_CROSS_FEEDBACK;
  float prev_cross_cos_ = 1.0f;
  float prev_raw_wet_gain_ = 1.0f;
  float prev_pan_gain_l_ = 0.5f, prev_pan_gain_r_ = 0.5f;
  float prev_filter_gain_l_[8] = {0.0f};
  float prev_filter_gain_r_[8] = {0.0f};
  float prev_filter_range_ = 0.0f;
  float prev_verb_mix_ = 0.0f;
  bool effects_ramping_initialized_ = false;

  // Scratch Arrays (Member variables to guarantee zero stack allocation)
  StereoInputSummer input_summer_;
  float mono_in_[BLOCK_SIZE] = {0.0f};
  float granularBlockL_[BLOCK_SIZE] = {0.0f};
  float granularBlockR_[BLOCK_SIZE] = {0.0f};
  float wet_l_chunk_[BLOCK_SIZE] = {0.0f};
  float wet_r_chunk_[BLOCK_SIZE] = {0.0f};
  float eff_in_l_[BLOCK_SIZE] = {0.0f};
  float eff_in_r_[BLOCK_SIZE] = {0.0f};
  float filtered_l_[BLOCK_SIZE] = {0.0f};
  float filtered_r_[BLOCK_SIZE] = {0.0f};
  float raw_wet_l_[BLOCK_SIZE] = {0.0f};
  float raw_wet_r_[BLOCK_SIZE] = {0.0f};

  // Diffusers & Reverb Tank
  TankDelay tank_l_;
  TankDelay tank_r_;
  SchroederDiffuser pre_diff_l_[4];
  SchroederDiffuser pre_diff_r_[4];
  CloudAllpass cloud_l_[6];
  CloudAllpass cloud_r_[6];
  float allpass_pool_[AllpassPoolSize()];
};

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif // AUDIO_ENGINE_H
