#pragma once
#ifndef REALTIME_GRANULATOR_H
#define REALTIME_GRANULATOR_H

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

#include "GranularBuffer.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace landscape {

constexpr int MAX_VOICES = 32;
constexpr int MAX_GRAIN_SAMPLES = 36000; // 750 ms per pass (1.5 s per 2-pass voice in 96k buffer)
constexpr int MIN_GRAIN_SAMPLES = 96;    // 2 ms minimum grain duration
constexpr int TUKEY_LUT_SIZE = 512;
constexpr float TUKEY_TAPER = 0.70f;
constexpr int RELEASE_SAMPLES = 64;      // 1.3 ms fade-out for a grain that must stop early

enum class ArpScaleMode {
  FIFTHS = 0,    // Switch 1 UP: 13-step open fifths, fourths, and octaves
  CHROMATIC = 1, // Switch 1 CENTER: 10-step lush Pentatonic / Add9 consonant harmony
  CUSTOM = 2     // Switch 1 DOWN: generative dual-LFO octave + Just Intonation intervals
};

using ArpMode = ArpScaleMode;

/**
 * @brief State structure for an individual synchronous grain voice.
 * Fully initialized default values to satisfy strict type safety and UB prevention.
 */
struct GrainVoice {
  bool active = false;
  float read_delay = 0.0f;       // Current delay in samples behind write head
  float start_delay_pos = 0.0f;  // Initial delay captured at spawn (for repeat passes)
  float rate = 1.0f;             // Playback rate ratio (>0 forward, <0 reverse)
  int length = 0;                // Total duration of grain in samples
  int age = 0;                   // Current sample position [0, length)
  int lifespan = 1;              // Number of play cycles remaining (MIN_GRAIN_PLAYS = 2)
  float pan_l = 0.7071f;         // Equal-power left panning gain
  float pan_r = 0.7071f;         // Equal-power right panning gain
  float window_phase = 0.0f;     // Phase in Tukey window [0.0, 1.0)
  float window_step = 0.0f;      // Window phase increment per sample
  int start_delay = 0;           // Onset stagger delay (0..47 samples)
  int release_left = 0;          // Samples left in a forced fade-out (0 = not releasing)
  float release_gain = 1.0f;     // Fade-out gain, reaches 0.0 when the release ends
};

/**
 * @brief Deterministic, synchronous 32-voice real-time granulator.
 * Renders all active voices synchronously within audio blocks without asynchronous
 * loop() preparation, multi-queue state machines, or shared volatile flags.
 */
class RealtimeGranulator {
public:
  RealtimeGranulator() = default;
  ~RealtimeGranulator() = default;

#if defined(__GNUC__) && !defined(__clang__)
  __attribute__((optimize("Os")))
#endif
  void Init(float sample_rate = 48000.0f, const GranularBuffer *buffer = nullptr) {
    sample_rate_ = sample_rate;
    buffer_ = buffer;
    GenerateTukeyLut(TUKEY_TAPER);
    Reset();
    post_dc_l_.Init(sample_rate);
    post_dc_r_.Init(sample_rate);
  }

  void SetBuffer(const GranularBuffer *buffer) {
    buffer_ = buffer;
  }

#if defined(__GNUC__) && !defined(__clang__)
  __attribute__((optimize("Os")))
#endif
  void Reset() {
    for (int i = 0; i < MAX_VOICES; ++i) {
      voices_[i] = GrainVoice{};
    }
    active_mask_ = 0u;
    spawn_phase_ = 0.0f;
    next_spawn_scale_ = 1.0f;
    avg_length_scale_ = 1.0f;
    target_overlap_ = 8.0f;
    max_active_voices_ = 16;
    smoothed_density_gain_ = 1.0f;
    target_density_gain_ = 1.0f;
    spawn_rng_ = 0x12345678;
    last_buffer_writes_ = 0;
    writes_initialized_ = false;
    freeze_cursor_ = 0.0f;
    cached_size_norm_ = -1.0f;
    cached_density_ = -1.0f;
    post_dc_l_.Reset();
    post_dc_r_.Reset();
  }

  /**
   * @brief Configures granular synthesis parameters with ergonomic potentiometer mappings.
   * @param grain_size_norm Normalized grain size [0.0, 1.0] (mapped via powf(size, 1.4) from 80ms to 512ms).
   * @param grain_density_norm Normalized grain density [0.0, 1.0] (maps overlap from 1 to 32 grains).
   * @param grain_arp_norm Arpeggio reach [0.0, 1.0].
   * @param randomness_norm Randomness amount [0.0, 1.0] (modulates jitter, detune, intervals, and freeze window).
   * @param arp_mode Pitch quantization scale mode (FIFTHS, CHROMATIC, CUSTOM).
   * @param arp_lfo1 Breathing LFO 1 modulation.
   * @param arp_lfo2 Breathing LFO 2 modulation.
   */
  void SetParameters(float grain_size_norm, float grain_density_norm, float grain_arp_norm,
                     float randomness_norm, ArpScaleMode arp_mode,
                     float arp_lfo1, float arp_lfo2) {
    float size_clamped = std::max(0.0f, std::min(1.0f, grain_size_norm));
    if (size_clamped != cached_size_norm_) {
      cached_size_norm_ = size_clamped;
      constexpr float kMinSize = 0.10666667f; // 80 ms @ 48 kHz (3840 / 36000 samples)
      constexpr float kMaxSize = 1.0f;        // 750 ms @ 48 kHz (36000 samples, 1.5 s over 2 passes)
      float curve = std::pow(size_clamped, 1.4f);
      cached_physical_size_ = kMinSize + curve * (kMaxSize - kMinSize);
    }
    UpdateParameters(cached_physical_size_, grain_density_norm, grain_arp_norm, randomness_norm,
                     arp_mode, arp_lfo1, arp_lfo2);
  }

  void UpdateParameters(float grain_size, float grain_density, float arp_reach,
                        float randomness, ArpScaleMode scale_mode,
                        float arp_lfo1, float arp_lfo2) {
    float clamped_size = std::max(0.05f, std::min(1.0f, grain_size));
    float clamped_density = std::max(0.0f, std::min(1.0f, grain_density));
    grain_size_ = clamped_size;
    grain_density_ = clamped_density;
    arp_reach_ = std::max(0.0f, std::min(1.0f, arp_reach));
    randomness_ = std::max(0.0f, std::min(1.0f, randomness));
    scale_mode_ = scale_mode;
    arp_lfo1_ = arp_lfo1;
    arp_lfo2_ = arp_lfo2;

    if (clamped_density != cached_density_) {
      cached_density_ = clamped_density;
      // Density target overlap: exponential mapping from 1 to 32 grains
      target_overlap_ = std::pow(32.0f, std::pow(grain_density_, 0.8f));
      max_active_voices_ = static_cast<int>(std::fmin(32.0f, target_overlap_ * 1.5f + 1.5f));
      if (max_active_voices_ > MAX_VOICES) max_active_voices_ = MAX_VOICES;
      if (max_active_voices_ < 1) max_active_voices_ = 1;

      // Density loudness trim: uncorrelated grains sum in power (sqrt(N)), so equal-power
      // scaling keeps the cloud level flat across the Density knob.
      constexpr float kDensityTrim = 0.5f;
      constexpr float kWindowPower = (1.0f - TUKEY_TAPER) + TUKEY_TAPER * 0.375f; // 0.5625
      target_density_gain_ = std::pow(target_overlap_ / 16.0f, -kDensityTrim) *
                             std::sqrt((1.0f / 9.0f) / kWindowPower);
    }
  }

  /**
   * @brief Chance [0.0, 1.0] that a newly spawned grain plays reversed (0 = all forward, 1 = all reversed).
   */
  void SetReverseProbability(float prob) {
    reverse_prob_ = std::max(0.0f, std::min(1.0f, prob));
  }

  /**
   * @brief Synchronously renders active grain voices directly from GranularBuffer into stereo blocks.
   */
  void ProcessBlockStereo(const GranularBuffer &buffer, float *out_l, float *out_r, size_t size) {
    std::memset(out_l, 0, size * sizeof(float));
    std::memset(out_r, 0, size * sizeof(float));

    if (!buffer.IsWarm()) {
      return;
    }

    // Track buffer write-head advances so active grains stay locked to continuous audio
    const uint64_t cur_writes = buffer.GetTotalWrites();
    uint64_t delta_writes = 0;
    if (!writes_initialized_ || buffer_ != &buffer || cur_writes < last_buffer_writes_) {
      buffer_ = &buffer;
      last_buffer_writes_ = cur_writes;
      writes_initialized_ = true;
    } else {
      delta_writes = cur_writes - last_buffer_writes_;
      last_buffer_writes_ = cur_writes;
    }

    const float max_valid_delay = buffer.HasWrapped()
        ? static_cast<float>(GRANULAR_BUFFER_SIZE - 4)
        : std::max(4.0f, static_cast<float>(buffer.GetSamplesRecorded()) - 4.0f);
    // Write-head advance per output sample: 1 while recording, 0 while frozen
    const float write_rate = (size > 0) ? static_cast<float>(delta_writes) / static_cast<float>(size) : 0.0f;
    const bool limit_grows = !buffer.HasWrapped();

    if (delta_writes > 0 && active_mask_ != 0u) {
      const float delta_f = static_cast<float>(delta_writes);
      uint32_t mask = active_mask_;
      while (mask != 0u) {
        const int v = __builtin_ctz(mask);
        mask &= (mask - 1u);
        GrainVoice &g = voices_[v];
        g.read_delay += delta_f;
        g.start_delay_pos += delta_f;
        if (g.read_delay < 4.0f || g.read_delay > max_valid_delay) {
          DeactivateVoice(v);
        }
      }
    }

    // 1. Synchronously schedule grain spawning
    ScheduleGrainSpawning(buffer, size);

    // 2. Render all active voices directly from circular buffer (chunked zero-branch inner loop)
    constexpr uint32_t kLutMaxFx = static_cast<uint32_t>(TUKEY_LUT_SIZE - 1) << 23;
    uint32_t render_mask = active_mask_;
    while (render_mask != 0u) {
      const int v = __builtin_ctz(render_mask);
      render_mask &= (render_mask - 1u);
      GrainVoice &g = voices_[v];
      // Samples are read as raw int16 values, so the pan gains carry the int16 -> float scale
      const float pl = g.pan_l * GRAIN_S16_INV_SCALE;
      const float pr = g.pan_r * GRAIN_S16_INV_SCALE;

      size_t i = 0;
      if (g.start_delay > 0) {
        const size_t delay_samples = static_cast<size_t>(g.start_delay);
        if (delay_samples >= size) {
          g.start_delay -= static_cast<int>(size);
          continue;
        }
        i = delay_samples;
        g.start_delay = 0;
      }

      // A grain about to run off the recorded audio (e.g. a 2 s freeze window caught by the
      // resumed write head) fades out on the spot instead of being cut at full window gain.
      if (g.release_left > 0) {
        g.rate = write_rate;
      } else if (RunsOffBuffer(g, max_valid_delay, write_rate, limit_grows, size)) {
        StartRelease(g, write_rate);
      }

      while (i < size && g.active) {
        if (g.read_delay < 4.0f || g.read_delay > max_valid_delay) {
          DeactivateVoice(v);
          break;
        }

        const size_t rem_block = size - i;
        const size_t rem_grain = (g.age < g.length) ? static_cast<size_t>(g.length - g.age) : 0;
        if (rem_grain == 0) {
          DeactivateVoice(v);
          break;
        }
        size_t n_chunk = (rem_block < rem_grain) ? rem_block : rem_grain;
        const bool releasing = g.release_left > 0;
        if (releasing && static_cast<size_t>(g.release_left) < n_chunk) {
          n_chunk = static_cast<size_t>(g.release_left);
        }

        // Bound chunk length so read_delay stays inside [4.0f, max_valid_delay]
        if (g.rate > 0.0f) {
          const float end_d = g.read_delay - static_cast<float>(n_chunk - 1) * g.rate;
          if (end_d < 4.0f) {
            const size_t safe_n = static_cast<size_t>((g.read_delay - 4.0f) / g.rate) + 1;
            if (safe_n < n_chunk) n_chunk = safe_n;
          }
        } else if (g.rate < 0.0f) {
          const float end_d = g.read_delay - static_cast<float>(n_chunk - 1) * g.rate;
          if (end_d > max_valid_delay) {
            const size_t safe_n = static_cast<size_t>((max_valid_delay - g.read_delay) / (-g.rate)) + 1;
            if (safe_n < n_chunk) n_chunk = safe_n;
          }
        }

        float d = g.read_delay;
        uint32_t wp_fx = static_cast<uint32_t>(std::fmin(g.window_phase, 1.0f) * static_cast<float>(kLutMaxFx));
        const uint32_t wp_step_fx = static_cast<uint32_t>(g.window_step * static_cast<float>(kLutMaxFx));
        if (releasing) {
          RenderRun<true>(buffer, out_l + i, out_r + i, n_chunk, d, g.rate, wp_fx, wp_step_fx,
                          pl, pr, g.release_gain);
        } else {
          RenderRun<false>(buffer, out_l + i, out_r + i, n_chunk, d, g.rate, wp_fx, wp_step_fx,
                           pl, pr, g.release_gain);
        }

        g.read_delay = d;
        g.window_phase += static_cast<float>(n_chunk) * g.window_step;
        g.age += static_cast<int>(n_chunk);
        i += n_chunk;

        if (releasing) {
          g.release_left -= static_cast<int>(n_chunk);
          if (g.release_left <= 0 || g.age >= g.length) {
            DeactivateVoice(v);
            break;
          }
        }

        if (g.age >= g.length) {
          g.lifespan--;
          if (g.lifespan > 0) {
            // Repeat pass: restart window and age, reset delay to captured start
            g.age = 0;
            g.window_phase = 0.0f;
            g.read_delay = g.start_delay_pos;
            // The window is at zero here, so dropping a pass that would run off the buffer is silent
            if (g.read_delay < 4.0f || g.read_delay > max_valid_delay ||
                RunsOffBuffer(g, max_valid_delay, write_rate, limit_grows, size)) {
              DeactivateVoice(v);
              break;
            }
          } else {
            DeactivateVoice(v);
            break;
          }
        } else if (d < 4.0f || d > max_valid_delay) {
          DeactivateVoice(v);
          break;
        }
      }
    }

    // 3. Apply smoothed density auto-gain & post-granulator DC blocking
    float xl = post_dc_l_.x_, yl = post_dc_l_.y_, xr = post_dc_r_.x_, yr = post_dc_r_.y_;
    const float rl = post_dc_l_.r_, rr = post_dc_r_.r_;
    if (std::fabs(target_density_gain_ - smoothed_density_gain_) < 1e-6f) {
      smoothed_density_gain_ = target_density_gain_;
      const float gain = smoothed_density_gain_;
      for (size_t i = 0; i < size; ++i) {
        out_l[i] = BufferDcBlocker::Step(out_l[i] * gain, xl, yl, rl);
        out_r[i] = BufferDcBlocker::Step(out_r[i] * gain, xr, yr, rr);
      }
    } else {
      const float target = target_density_gain_;
      float gain = smoothed_density_gain_;
      for (size_t i = 0; i < size; ++i) {
        gain += 0.005f * (target - gain);
        out_l[i] = BufferDcBlocker::Step(out_l[i] * gain, xl, yl, rl);
        out_r[i] = BufferDcBlocker::Step(out_r[i] * gain, xr, yr, rr);
      }
      smoothed_density_gain_ = gain;
    }
    post_dc_l_.x_ = xl;
    post_dc_l_.y_ = FlushDenormal(yl);
    post_dc_r_.x_ = xr;
    post_dc_r_.y_ = FlushDenormal(yr);
  }

  void ProcessBlockStereo(float *out_l, float *out_r, size_t size) {
    if (buffer_ != nullptr) {
      ProcessBlockStereo(*buffer_, out_l, out_r, size);
    } else {
      std::memset(out_l, 0, size * sizeof(float));
      std::memset(out_r, 0, size * sizeof(float));
    }
  }

  // --- Voice and Parameter Accessors ---
  inline int GetActiveVoiceCount() const {
    return static_cast<int>(__builtin_popcount(active_mask_));
  }

  inline float GetTargetOverlap() const { return target_overlap_; }
  inline float GetTargetDensityGain() const { return target_density_gain_; }
  inline float GetSmoothedDensityGain() const { return smoothed_density_gain_; }
  inline const GrainVoice &GetVoice(int idx) const { return voices_[idx]; }
  inline float GetReverseProbability() const { return reverse_prob_; }
  inline float GetAverageLengthScale() const { return avg_length_scale_; }

  inline float EvaluateTukeyWindow(float phase) const {
    if (phase <= 0.0f) return tukey_lut_[0];
    if (phase >= 1.0f) return tukey_lut_[TUKEY_LUT_SIZE - 1];
    float pos = phase * static_cast<float>(TUKEY_LUT_SIZE - 1);
    int idx = static_cast<int>(pos);
    float frac = pos - static_cast<float>(idx);
    if (idx >= TUKEY_LUT_SIZE - 1) return tukey_lut_[TUKEY_LUT_SIZE - 1];
    return tukey_lut_[idx] + frac * (tukey_lut_[idx + 1] - tukey_lut_[idx]);
  }

private:
  void GenerateTukeyLut(float taper) {
    for (int i = 0; i < TUKEY_LUT_SIZE; ++i) {
      float x = static_cast<float>(i) / static_cast<float>(TUKEY_LUT_SIZE - 1);
      if (x < 0.5f * taper) {
        tukey_lut_[i] = 0.5f * (1.0f - std::cos(2.0f * 3.14159265358979323846f * x / taper));
      } else if (x <= 1.0f - 0.5f * taper) {
        tukey_lut_[i] = 1.0f;
      } else {
        tukey_lut_[i] = 0.5f * (1.0f - std::cos(2.0f * 3.14159265358979323846f * (1.0f - x) / taper));
      }
    }
    // Strict boundary suppression (< -94 dBFS) + safety sentinel padding for unchecked chunk reads
    tukey_lut_[0] = 0.0f;
    tukey_lut_[TUKEY_LUT_SIZE - 1] = 0.0f;
    tukey_lut_[TUKEY_LUT_SIZE] = 0.0f;
    tukey_lut_[TUKEY_LUT_SIZE + 1] = 0.0f;
    for (int i = 0; i < TUKEY_LUT_SIZE + 1; ++i) {
      tukey_diff_[i] = tukey_lut_[i + 1] - tukey_lut_[i];
    }
    tukey_diff_[TUKEY_LUT_SIZE + 1] = 0.0f;
  }

  inline float FastRandFloat() {
    spawn_rng_ = spawn_rng_ * 1664525u + 1013904223u;
    return static_cast<float>(spawn_rng_ >> 8) * (1.0f / 16777216.0f);
  }

  inline void DeactivateVoice(int v) {
    voices_[v].active = false;
    active_mask_ &= ~(1u << v);
  }

  /**
   * @brief Renders one contiguous run of a voice into the output blocks.
   * pl/pr already include the int16 -> float scale (GRAIN_S16_INV_SCALE).
   * kFade ramps release_gain down by 1/RELEASE_SAMPLES per sample.
   */
  template <bool kFade>
  inline void RenderRun(const GranularBuffer &buffer, float *__restrict dl, float *__restrict dr,
                        size_t n, float &delay, float d_step, uint32_t &wp_fx,
                        uint32_t wp_step_fx, float pl, float pr, float &fade) const {
    // Furthest delay this run reads (+2 covers the second interpolation tap and float rounding).
    // Only a run that reaches past the top of the ring needs per-sample index wrapping.
    const int32_t head = buffer.GetWriteHeadPos();
    const float d_far = std::fmax(delay, delay - static_cast<float>(n) * d_step) + 2.0f;
    if (static_cast<float>(head) + d_far < static_cast<float>(GRANULAR_BUFFER_SIZE)) {
      RenderSpan<kFade, false>(buffer.Data(), head, dl, dr, n, delay, d_step, wp_fx, wp_step_fx,
                               pl, pr, fade);
    } else {
      RenderSpan<kFade, true>(buffer.Data(), head, dl, dr, n, delay, d_step, wp_fx, wp_step_fx,
                              pl, pr, fade);
    }
  }

  template <bool kFade, bool kWrap>
  inline void RenderSpan(const int16_t *line, int32_t head, float *__restrict dl,
                         float *__restrict dr, size_t n, float &delay, float d_step,
                         uint32_t &wp_fx, uint32_t wp_step_fx, float pl, float pr,
                         float &fade) const {
    constexpr float kFadeStep = 1.0f / static_cast<float>(RELEASE_SAMPLES);
    constexpr float kPhaseScale = 1.0f / 8388608.0f; // window phase fraction bits -> [0, 1)
    constexpr int32_t kSize = static_cast<int32_t>(GRANULAR_BUFFER_SIZE);
    const int16_t *base = line + head;
    const float win_phase_step = static_cast<float>(wp_step_fx) * kPhaseScale;
    float d = delay;
    uint32_t wp = wp_fx;
    float f = fade;
    size_t k = 0;
    while (k < n) {
      // The window is linear inside each LUT segment: evaluate it once per segment, then step it
      const uint32_t idx = wp >> 23;
      const uint32_t frac_fx = wp & 0x7FFFFFu;
      size_t seg = n - k;
      if (wp_step_fx > 0u) {
        const uint32_t to_next = 0x800000u - frac_fx;
        const size_t in_segment = (to_next + wp_step_fx - 1u) / wp_step_fx;
        if (in_segment < seg) seg = in_segment;
      }
      float win = tukey_lut_[idx] + static_cast<float>(frac_fx) * kPhaseScale * tukey_diff_[idx];
      const float win_step = tukey_diff_[idx] * win_phase_step;
      float *__restrict out_l = dl + k;
      float *__restrict out_r = dr + k;
      for (size_t j = 0; j < seg; ++j) {
        float w = win;
        if (kFade) {
          f -= kFadeStep;
          w *= f;
        }
        const int32_t di = static_cast<int32_t>(d);
        const float d_frac = d - static_cast<float>(di);
        int a, b;
        if (kWrap) {
          int32_t i0 = head + di;
          if (i0 >= kSize) i0 -= kSize;
          const int32_t i1 = (i0 + 1 >= kSize) ? 0 : i0 + 1;
          a = line[i0];
          b = line[i1];
        } else {
          a = base[di];
          b = base[di + 1];
        }
        const float val = (a + (b - a) * d_frac) * w;
        out_l[j] += val * pl;
        out_r[j] += val * pr;
        d -= d_step;
        win += win_step;
      }
      wp += static_cast<uint32_t>(seg) * wp_step_fx;
      k += seg;
    }
    delay = d;
    wp_fx = wp;
    fade = f;
  }

  /**
   * @brief True when the voice's current pass will leave the recorded audio [4, max_valid_delay]
   * and is already close enough to that edge that it must start fading now.
   */
  static bool RunsOffBuffer(const GrainVoice &g, float max_valid_delay, float write_rate,
                            bool limit_grows, size_t block_size) {
    const float drift = write_rate - g.rate; // read delay change per output sample
    const float remaining = static_cast<float>(g.length - g.age);
    const float zone = static_cast<float>(block_size) * (std::fabs(g.rate) + 2.0f) + 8.0f;
    if (drift < 0.0f) {
      const float dist = g.read_delay - 4.0f; // closing on the write head
      return dist <= zone && dist < -drift * remaining;
    }
    // Before the buffer wraps, the oldest valid audio recedes as fast as the write head moves
    const float closing = drift - (limit_grows ? write_rate : 0.0f);
    if (closing > 0.0f) {
      const float dist = max_valid_delay - g.read_delay; // closing on audio being overwritten
      return dist <= zone && dist < closing * remaining;
    }
    return false;
  }

  static void StartRelease(GrainVoice &g, float write_rate) {
    g.release_left = RELEASE_SAMPLES;
    g.release_gain = 1.0f;
    g.rate = write_rate;
  }

  void ScheduleGrainSpawning(const GranularBuffer &buffer, size_t block_size) {
    int active_count = static_cast<int>(__builtin_popcount(active_mask_));

    // Nominal spawn interval in samples; actual per-grain length scaling is folded into next_spawn_scale_
    float base_length = grain_size_ * static_cast<float>(MAX_GRAIN_SAMPLES);
    constexpr float kPlaysPerVoice = 2.0f; // 2 repeat passes per voice
    float nominal_interval = std::max(1.0f, base_length * kPlaysPerVoice / target_overlap_);

    spawn_phase_ += static_cast<float>(block_size) / nominal_interval;
    if (active_count == 0 && spawn_phase_ < next_spawn_scale_) {
      // Never sit in dead silence with 0 active voices (e.g. right after warmup or if a
      // live grain hit the stopped write head when Freeze was engaged mid-grain).
      spawn_phase_ = next_spawn_scale_;
    }
    const float timing_jitter = 0.10f + 0.40f * grain_density_;
    int loop_guard = 0;
    while (spawn_phase_ >= next_spawn_scale_ && loop_guard++ < MAX_VOICES) {
      if (active_count >= max_active_voices_) {
        spawn_phase_ = next_spawn_scale_;
        break;
      }
      const uint32_t free_mask = ~active_mask_;
      if (free_mask == 0u) {
        spawn_phase_ = next_spawn_scale_;
        break;
      }
      spawn_phase_ -= next_spawn_scale_;
      const int slot = __builtin_ctz(free_mask);
      const float length_scale = SpawnVoice(voices_[slot], buffer);
      active_mask_ |= (1u << slot);
      active_count++;

      const float jitter = (1.0f - timing_jitter) + (2.0f * timing_jitter) * FastRandFloat();
      next_spawn_scale_ = std::max(0.05f, length_scale * jitter);
    }
    if (spawn_phase_ > next_spawn_scale_) {
      spawn_phase_ = next_spawn_scale_;
    }
  }

  float SpawnVoice(GrainVoice &v, const GranularBuffer &buffer) {
    v.active = true;
    v.age = 0;
    v.lifespan = 2; // MIN_GRAIN_PLAYS
    v.release_left = 0;
    v.release_gain = 1.0f;

    // 1. Calculate Pitch Ratio from Scales + Reach + Detune
    float semitones = ComputeGrainPitch();
    float rate = std::exp2(semitones * (1.0f / 12.0f));

    // 2. Playback Direction (Reverse probability)
    bool is_rev = (reverse_prob_ > 0.0f) && (FastRandFloat() < reverse_prob_);
    v.rate = is_rev ? -rate : rate;

    // 3. Length Calculation with Jitter and Inverse Pitch Scaling
    float len_jitter = 1.0f + 0.25f * (2.0f * FastRandFloat() - 1.0f);
    float len = grain_size_ * static_cast<float>(MAX_GRAIN_SAMPLES) * len_jitter;
    if (rate < 1.0f && rate > 0.01f) {
      float scale = std::fmin(4.0f, 1.0f / rate);
      len *= scale;
    }

    int32_t safe_offset = buffer.GetSafeWriteHeadOffset(grain_size_, std::fabs(v.rate));
    float candidate_delay = static_cast<float>(safe_offset);
    const float max_buf_delay = buffer.HasWrapped()
        ? static_cast<float>(GRANULAR_BUFFER_SIZE - 480)
        : std::max(480.0f, static_cast<float>(buffer.GetSamplesRecorded()) - 480.0f);

    if (buffer.IsHeld()) {
      // Freeze loop window: 0.25s .. 2.0s scaled by Random, capped to valid recorded audio
      const float target_window = (0.25f + randomness_ * 1.75f) * sample_rate_;
      const float freeze_loop_end = std::max(480.0f, std::min(max_buf_delay, target_window));
      const float loop_min = std::min(480.0f, freeze_loop_end * 0.10f);
      const float slice_span = std::max(static_cast<float>(MIN_GRAIN_SAMPLES), freeze_loop_end - loop_min);

      const float abs_rate = std::max(0.25f, std::fabs(v.rate));
      // Reserve at least 25% of the frozen slice for start-position scanning so long grains
      // play at full duration without getting chopped to a few ms near the slice boundaries.
      const float max_safe_len = std::max(static_cast<float>(MIN_GRAIN_SAMPLES),
                                          (0.75f * slice_span) / abs_rate);
      if (len > max_safe_len) {
        len = max_safe_len;
      }

      const float travel = std::min(slice_span, len * abs_rate);
      const float start_span = std::max(1.0f, slice_span - travel);
      const float min_start = (v.rate > 0.0f) ? (loop_min + travel) : loop_min;
      const float max_start = min_start + start_span;

      freeze_cursor_ += 240.0f;
      if (freeze_cursor_ >= start_span) {
        freeze_cursor_ = std::fmod(freeze_cursor_, start_span);
      }

      const float jitter = (FastRandFloat() - 0.5f) * randomness_ * start_span;
      candidate_delay = min_start + freeze_cursor_ + jitter;
      if (candidate_delay < min_start) candidate_delay = min_start;
      if (candidate_delay > max_start) candidate_delay = max_start;
    } else {
      if (v.rate < 0.0f) {
        // Reversed grains read away from the write head, so they need no forward-read
        // margin. Starting just behind the head leaves the whole buffer for their length.
        candidate_delay = static_cast<float>(SAFE_OFFSET_WARMUP);
      } else if (v.rate < 1.0f && !buffer.HasWrapped()) {
        const float early_cap = std::max(480.0f, max_buf_delay * 0.35f);
        if (candidate_delay > early_cap) {
          candidate_delay = early_cap;
        }
      }
      // Live playhead time-scrubbing: always guarantee the 960-sample (20 ms) minimum de-phasing
      // floor so overlapping unison grains never comb-filter, plus up to ~500 ms of history scatter
      // as Randomness rises (using at most 35% of remaining buffer room so grain length is never starved).
      const float avail_room = std::max(0.0f, max_buf_delay - candidate_delay);
      const float min_dephase = std::min(960.0f, avail_room);
      const float extra_scatter = std::min((randomness_ * randomness_) * 24000.0f, avail_room * 0.35f);
      const float jitter_room = std::max(min_dephase, extra_scatter);
      candidate_delay += FastRandFloat() * jitter_room;
      if (candidate_delay < 480.0f) candidate_delay = 480.0f;
      if (candidate_delay > max_buf_delay) candidate_delay = max_buf_delay;

      if (v.rate > 1.0f) {
        const float net_catchup = std::max(0.35f, v.rate - 0.65f);
        const float max_safe_len = std::max(static_cast<float>(MIN_GRAIN_SAMPLES),
                                            (candidate_delay - 480.0f) / net_catchup);
        if (len > max_safe_len) {
          len = max_safe_len;
        }
      } else if (v.rate < 1.0f) {
        const float max_safe_len = std::max(static_cast<float>(MIN_GRAIN_SAMPLES),
                                            (max_buf_delay - candidate_delay) / std::max(0.25f, 2.0f - v.rate));
        if (len > max_safe_len) {
          len = max_safe_len;
        }
      }
    }

    v.length = std::max(MIN_GRAIN_SAMPLES, std::min(static_cast<int>(len), MAX_GRAIN_SAMPLES));

    // Track the real grain length (pitch scaling plus freeze-window and buffer-edge caps)
    // so ScheduleGrainSpawning keeps the actual overlap at the Density target.
    const float nominal_len = std::max(1.0f, grain_size_ * static_cast<float>(MAX_GRAIN_SAMPLES));
    const float length_scale = static_cast<float>(v.length) / nominal_len;
    avg_length_scale_ += 0.25f * (length_scale - avg_length_scale_);
    v.window_phase = 0.0f;
    v.window_step = (v.length > 1) ? (1.0f / static_cast<float>(v.length - 1)) : 0.0f;

    // 4. Equal-Power Stereo Panning
    // Spread widens with Density so sparse 1-voice streams follow AudioEngine's panLfo
    // without random grains getting muted on the opposite channel.
    float pan = 0.5f + (FastRandFloat() - 0.5f) * grain_density_;
    float angle = pan * (3.14159265358979323846f * 0.5f);
    v.pan_l = std::cos(angle);
    v.pan_r = std::sin(angle);

    // 5. Onset Stagger (0..47 samples)
    v.start_delay = static_cast<int>(FastRandFloat() * 47.9f);

    // 6. Start Delay Distance behind Write Head
    v.read_delay = candidate_delay;
    v.start_delay_pos = v.read_delay;
    return length_scale;
  }

  float ComputeGrainPitch() {
    float pitch = 0.0f;
    float reach = std::pow(arp_reach_, 0.65f);
    float lfo1_norm = (arp_lfo1_ >= 0.0f && arp_lfo1_ <= 1.0f) ? arp_lfo1_ : (0.5f * (arp_lfo1_ + 1.0f));
    float breathe1 = 0.5f + 0.5f * std::max(0.0f, std::min(1.0f, lfo1_norm));

    // Switch 1 UP (FIFTHS): Open 5ths, 4ths, and octaves
    static const float kFifths[13] = {
      0.0f, 12.0f, 7.0f, 12.0f, -12.0f, 19.0f, 7.0f, -5.0f, 24.0f, 12.0f, 19.0f, 7.0f, 24.0f
    };
    // Switch 1 CENTER (CHROMATIC / Lush Pentatonic-Add9): Warm 3rds, 5ths, 6ths, 9ths, and octaves
    static const float kPentatonicAdd9[10] = {
      0.0f, 12.0f, 7.0f, -12.0f, 4.0f, 9.0f, 14.0f, -5.0f, 19.0f, 24.0f
    };
    // Switch 1 DOWN (CUSTOM): Dual-LFO octave registers + Just Intonation intervals
    static const float kOctaves[21] = {
      0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 12.0f, -12.0f, 12.0f, -12.0f, 12.0f,
      24.0f, 12.0f, -12.0f, 24.0f, 12.0f, -12.0f, 24.0f, 12.0f, 24.0f, -12.0f, 24.0f
    };
    static const float kIntervals[18] = {
      0.0f, 7.02f, -7.02f, 7.02f, 4.98f, 7.02f, -7.02f, 7.02f, 4.98f,
      7.02f, 3.86f, -7.02f, 7.02f, -7.02f, 4.98f, 8.84f, 7.02f, -7.02f
    };

    if (reach > 1.0e-4f) {
      // Randomness widens the effective LFO breathe toward full reach without breaking scale quantization
      const float eff_breathe1 = std::min(1.0f, breathe1 + randomness_ * (1.0f - breathe1));
      if (scale_mode_ == ArpScaleMode::FIFTHS) {
        pitch = kFifths[PickScaleIndex(12.0f * reach * eff_breathe1, 13)];
      } else if (scale_mode_ == ArpScaleMode::CHROMATIC) {
        pitch = kPentatonicAdd9[PickScaleIndex(9.0f * reach * eff_breathe1, 10)];
      } else {
        float lfo2_norm = (arp_lfo2_ >= 0.0f && arp_lfo2_ <= 1.0f) ? arp_lfo2_ : (0.5f * (arp_lfo2_ + 1.0f));
        float breathe2 = 0.5f + 0.5f * std::max(0.0f, std::min(1.0f, lfo2_norm));
        const float eff_breathe2 = std::min(1.0f, breathe2 + randomness_ * (1.0f - breathe2));
        float oct = kOctaves[PickScaleIndex(20.0f * reach * eff_breathe1, 21)];
        float inter = kIntervals[PickScaleIndex(17.0f * reach * eff_breathe2, 18)];
        pitch = oct + inter;
      }

      // Octave register leap scaled by both Randomness and Arp Reach (never fires at Arp Reach = 0%)
      if (randomness_ > 0.25f && FastRandFloat() < (randomness_ * reach * 0.60f)) {
        pitch += (FastRandFloat() < 0.5f) ? 12.0f : -12.0f;
      }

      // Octave folding to [-12.0, +24.0]
      while (pitch > 24.0f) pitch -= 12.0f;
      while (pitch < -12.0f) pitch += 12.0f;
    }

    // Micro-detune +/-4 to +/-15 cents (0.04 to 0.15 semitones)
    float detune_depth = 0.04f + (0.15f - 0.04f) * randomness_;
    float detune = detune_depth * (2.0f * FastRandFloat() - 1.0f);
    pitch += detune;

    return pitch;
  }

  inline int PickScaleIndex(float reach, int size) {
    if (!(reach > 0.0f)) return 0;
    float u = FastRandFloat();
    int idx = static_cast<int>(u * (reach + 1.0f));
    int top = static_cast<int>(std::ceil(reach));
    if (idx > top) idx = top;
    return (idx < size - 1) ? idx : (size - 1);
  }

  GrainVoice voices_[MAX_VOICES];
  uint32_t active_mask_ = 0u;
  BufferDcBlocker post_dc_l_;
  BufferDcBlocker post_dc_r_;
  const GranularBuffer *buffer_ = nullptr;
  float tukey_lut_[TUKEY_LUT_SIZE + 2];
  float tukey_diff_[TUKEY_LUT_SIZE + 2];
  float sample_rate_ = 48000.0f;
  float grain_size_ = 0.25f;
  float grain_density_ = 0.5f;
  float cached_size_norm_ = -1.0f;
  float cached_physical_size_ = 0.25f;
  float cached_density_ = -1.0f;
  float arp_reach_ = 0.0f;
  float randomness_ = 0.0f;
  ArpScaleMode scale_mode_ = ArpScaleMode::FIFTHS;
  float arp_lfo1_ = 0.0f;
  float arp_lfo2_ = 0.0f;
  float target_overlap_ = 8.0f;
  int max_active_voices_ = 16;
  float spawn_phase_ = 0.0f;
  float next_spawn_scale_ = 1.0f;
  float avg_length_scale_ = 1.0f;
  float reverse_prob_ = 0.0f;
  float target_density_gain_ = 1.0f;
  float smoothed_density_gain_ = 1.0f;
  uint32_t spawn_rng_ = 0x12345678;
  uint64_t last_buffer_writes_ = 0;
  bool writes_initialized_ = false;
  float freeze_cursor_ = 0.0f;
};

} // namespace landscape

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif // REALTIME_GRANULATOR_H
