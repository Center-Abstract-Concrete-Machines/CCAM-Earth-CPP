#pragma once
#ifndef GRANULAR_BUFFER_H
#define GRANULAR_BUFFER_H

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Settings.h"

namespace landscape {

// Buffer Configuration Constants
constexpr size_t GRANULAR_BUFFER_SIZE = MAX_BUFFER_SIZE; // 96,000 samples (2.0 s @ 48 kHz)
constexpr float GRAIN_S16_SCALE = 16384.0f;
constexpr float GRAIN_S16_INV_SCALE = 1.0f / 16384.0f;
constexpr int RESUME_FADE_SAMPLES = RESUME_CROSSFADE_SAMPLES; // 2 ms click-free unfreeze blend
constexpr float OVERDUB_FEEDBACK = FREEZE_OVERDUB_FEEDBACK;
constexpr float DEFAULT_RECORD_FEEDBACK = RECORD_FEEDBACK;
constexpr int SAFE_OFFSET_WARMUP = 480;       // 10 ms minimum write-head margin
constexpr size_t WARMUP_THRESHOLD = 4800;     // 100 ms buffer warmup threshold

// Denormal protection for recursive filter states: block loops can flush states once per block
// (on the Daisy the callback also runs with FPU flush-to-zero).
inline float FlushDenormal(float v) {
  return (std::fabs(v) < 1.0e-15f) ? 0.0f : v;
}

/**
 * @brief Subsonic 5.3 Hz Input DC Blocker.
 * Strips hardware ADC bias and DC offsets before circular buffer accumulation.
 * r = 1.0 - (2*pi*5.345 / fs) approx 0.9993 @ 48 kHz.
 */
struct BufferDcBlocker {
  float x_ = 0.0f;
  float y_ = 0.0f;
  float r_ = 0.9993f;

  void Init(float sample_rate = 48000.0f) {
    x_ = 0.0f;
    y_ = 0.0f;
    r_ = 1.0f - (2.0f * 3.14159265358979323846f * 5.345f / sample_rate);
    if (r_ < 0.99f) r_ = 0.99f;
    if (r_ > 0.9997f) r_ = 0.9997f;
  }

  void Reset() {
    x_ = 0.0f;
    y_ = 0.0f;
  }

  inline float Process(float in) {
    y_ = FlushDenormal(Step(in, x_, y_, r_));
    return y_;
  }

  // One sample on caller-held state without per-sample denormal branch (flush y once per block)
  static inline float Step(float in, float &x, float &y, float r) {
    const float out = in - x + r * y;
    x = in;
    y = out;
    return out;
  }
};

/**
 * @brief Recording ring for the granulator: 96,000 int16 samples (2.0 s, 187.5 KB in internal
 * AXI SRAM). The write head moves backwards (the newest sample sits just above it), so a read
 * at delay d behind the head is at index write_pos + d.
 */
class GranularBuffer {
public:
  GranularBuffer() = default;
  ~GranularBuffer() = default;

  void Init(float sample_rate = 48000.0f) {
    sample_rate_ = sample_rate;
    dc_blocker_.Init(sample_rate);
    Reset();
  }

  void Reset() {
    std::memset(line_, 0, sizeof(line_));
    write_pos_ = 0;
    dc_blocker_.Reset();
    samples_recorded_ = 0;
    has_wrapped_ = false;
    is_held_ = false;
    was_held_ = false;
    resume_counter_ = 0;
    last_frozen_sample_ = 0.0f;
    total_writes_ = 0;
  }

  // --- Sample Conversion Helpers ---
  // Saturating, rounds half away from zero, NaN -> 0. Branchless (VSEL/VMAXNM/VMINNM on the M7).
  static inline int16_t FloatToS16(float x) {
    float v = x * GRAIN_S16_SCALE;
    v = (v == v) ? v : 0.0f; // NaN guard
    v = std::fmin(std::fmax(v, -32768.0f), 32767.0f);
    return static_cast<int16_t>(v + std::copysign(0.5f, v));
  }

  static inline float S16ToFloat(int16_t s) {
    return static_cast<float>(s) * GRAIN_S16_INV_SCALE;
  }

  // --- Asymptotic Overdub Saturation ---
  static inline float OverdubLimit(float x) {
    const float a = std::fabs(x);
    if (!(a > 0.5f)) return x;
    const float u = a - 0.5f;
    const float y = 0.5f + 0.5f * u / (0.5f + u);
    return (x < 0.0f) ? -y : y;
  }

  inline float WrapDelay(float delay) const {
    constexpr float kBufferSize = static_cast<float>(GRANULAR_BUFFER_SIZE);
    if (delay >= 0.0f && delay < kBufferSize) {
      return delay;
    }
    float wrapped = std::fmod(delay, kBufferSize);
    if (wrapped < 0.0f) {
      wrapped += kBufferSize;
    }
    return wrapped;
  }

  inline float WrapPos(float pos) const {
    return WrapDelay(pos);
  }

  /**
   * @brief Writes one input sample (see WriteBlock()).
   */
  inline void WriteSample(float input, float record_feedback = DEFAULT_RECORD_FEEDBACK,
                          bool overdub = false, bool held = false) {
    const float eff_fb = (overdub && record_feedback <= DEFAULT_RECORD_FEEDBACK)
                             ? OVERDUB_FEEDBACK
                             : record_feedback;
    const bool apply_limit = overdub || (eff_fb > DEFAULT_RECORD_FEEDBACK + 1.0e-4f);
    WriteBlock(&input, 1, eff_fb, 0.0f, apply_limit, held);
  }

  /**
   * @brief Records a block: DC blocking, freeze hold, previous-pass feedback and overdub limiting.
   * The feedback ramps across the block: sample k uses fb + (k + 1) * fb_step.
   * While held the write head stands still (the DC blocker keeps tracking the input), and the
   * first 96 samples after a hold blend from the pre-freeze boundary sample into the live input.
   */
  void WriteBlock(const float *in, size_t n, float fb, float fb_step, bool overdub, bool held) {
    if (n == 0) {
      return;
    }
    if (held != was_held_) {
      if (held) {
        last_frozen_sample_ = S16ToFloat(line_[NextIndex(write_pos_)]); // newest recorded sample
      } else {
        resume_counter_ = RESUME_FADE_SAMPLES;
      }
      was_held_ = held;
    }
    is_held_ = held;

    float dc_x = dc_blocker_.x_, dc_y = dc_blocker_.y_;
    const float dc_r = dc_blocker_.r_;
    if (held) {
      for (size_t k = 0; k < n; ++k) {
        BufferDcBlocker::Step(in[k], dc_x, dc_y, dc_r);
      }
      dc_blocker_.x_ = dc_x;
      dc_blocker_.y_ = FlushDenormal(dc_y);
      return;
    }

    const bool limit_active = overdub || (fb > DEFAULT_RECORD_FEEDBACK + 1.0e-4f) ||
                              (fb + fb_step * static_cast<float>(n) > DEFAULT_RECORD_FEEDBACK + 1.0e-4f);
    size_t k = 0;
    size_t pos = write_pos_;
    for (; k < n && resume_counter_ > 0; ++k) {
      fb += fb_step;
      float dc_in = BufferDcBlocker::Step(in[k], dc_x, dc_y, dc_r);
      const float phase = static_cast<float>(RESUME_FADE_SAMPLES - resume_counter_) /
                          static_cast<float>(RESUME_FADE_SAMPLES);
      dc_in = last_frozen_sample_ + phase * (dc_in - last_frozen_sample_);
      const float eff_fb = fb * phase;
      resume_counter_--;
      float accumulated = dc_in + S16ToFloat(line_[pos]) * eff_fb;
      if (overdub || eff_fb > DEFAULT_RECORD_FEEDBACK + 1.0e-4f) {
        accumulated = OverdubLimit(accumulated);
      }
      line_[pos] = FloatToS16(accumulated);
      pos = (pos == 0) ? GRANULAR_BUFFER_SIZE - 1 : pos - 1;
    }
    while (k < n) {
      // Contiguous run down to index 0, then the head wraps to the top
      const size_t run = std::min(n - k, pos + 1);
      if (limit_active) {
        RecordRun<true>(in + k, run, fb, fb_step, dc_x, dc_y, dc_r, line_ + pos);
      } else {
        RecordRun<false>(in + k, run, fb, fb_step, dc_x, dc_y, dc_r, line_ + pos);
      }
      k += run;
      pos = (pos >= run) ? pos - run : GRANULAR_BUFFER_SIZE - 1;
    }
    write_pos_ = pos;
    dc_blocker_.x_ = dc_x;
    dc_blocker_.y_ = FlushDenormal(dc_y);

    total_writes_ += n;
    if (!has_wrapped_) {
      samples_recorded_ = std::min(GRANULAR_BUFFER_SIZE, samples_recorded_ + n);
      has_wrapped_ = (samples_recorded_ >= GRANULAR_BUFFER_SIZE);
    }
  }

  // --- Fractional Interpolated Delay Reads ---

  /**
   * @brief Reads from buffer using fractional sample delay behind the write head (Linear).
   */
  inline float ReadLinear(float delay_samples) const {
    return ReadLinearUnchecked(WrapDelay(delay_samples));
  }

  /**
   * @brief Reads from buffer using 4-point cubic Hermite interpolation.
   */
  inline float ReadHermite(float delay_samples) const {
    return ReadHermiteUnchecked(WrapDelay(delay_samples));
  }

  /**
   * @brief Unchecked fast delay reads when caller already guarantees delay in [0, GRANULAR_BUFFER_SIZE - 4].
   */
  inline float ReadLinearUnchecked(float delay_samples) const {
    const int32_t di = static_cast<int32_t>(delay_samples);
    const float frac = delay_samples - static_cast<float>(di);
    const int32_t i0 = WrapIndex(static_cast<int32_t>(write_pos_) + di);
    const int16_t a = line_[i0];
    const int16_t b = line_[WrapIndex(i0 + 1)];
    return S16ToFloat(static_cast<int16_t>(a + (b - a) * frac));
  }

  inline float ReadHermiteUnchecked(float delay_samples) const {
    const int32_t di = static_cast<int32_t>(delay_samples);
    const float f = delay_samples - static_cast<float>(di);
    const int32_t t = WrapIndex(static_cast<int32_t>(write_pos_) + di);
    const int16_t xm1 = line_[(t == 0) ? static_cast<int32_t>(GRANULAR_BUFFER_SIZE) - 1 : t - 1];
    const int16_t x0 = line_[t];
    const int16_t x1 = line_[WrapIndex(t + 1)];
    const int16_t x2 = line_[WrapIndex(t + 2)];
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float b_neg = w + a;
    return S16ToFloat(static_cast<int16_t>((((a * f) - b_neg) * f + c) * f + x0));
  }

  /**
   * @brief Reads from buffer at an absolute sample position with wrapping and Hermite interpolation.
   */
  inline float ReadAbsolute(float sample_pos) const {
    return ReadHermite(sample_pos - static_cast<float>(write_pos_));
  }

  // --- Safe Write Head Distance Protection ---
  inline int32_t GetSafeWriteHeadOffset(float grain_size_norm, float max_rate = 2.0f) const {
    float size_clamped = std::max(0.0f, std::min(1.0f, grain_size_norm));
    constexpr float kMaxGrainSamples = 36000.0f;
    constexpr float kMinGrainSamples = 96.0f;
    float effective_samples = std::max(kMinGrainSamples, size_clamped * kMaxGrainSamples);
    // In live mode the write head advances at 1.0 sample/sample, so a forward grain
    // only closes on the write head at rate (max_rate - 1.0), plus a 0.35x safety margin.
    float catchup_rate = std::max(0.35f, max_rate - 0.65f);
    float read_extent = effective_samples * catchup_rate;
    int32_t safe = static_cast<int32_t>(read_extent) + SAFE_OFFSET_WARMUP;
    if (safe < SAFE_OFFSET_WARMUP) safe = SAFE_OFFSET_WARMUP;
    constexpr int32_t kHalfBuffer = static_cast<int32_t>(GRANULAR_BUFFER_SIZE / 2);
    if (safe > kHalfBuffer) safe = kHalfBuffer;
    if (!has_wrapped_) {
      int32_t max_allowed = static_cast<int32_t>(samples_recorded_) - SAFE_OFFSET_WARMUP;
      if (safe > max_allowed) safe = max_allowed;
      if (safe < SAFE_OFFSET_WARMUP) safe = SAFE_OFFSET_WARMUP;
    }
    return safe;
  }

  inline int GetSafeOffset(float grain_size_fraction, float max_grain_size = 36000.0f) const {
    float size_clamped = std::max(0.0f, std::min(1.0f, grain_size_fraction));
    constexpr float kMinGrainSamples = 96.0f;
    float effective_samples = std::max(kMinGrainSamples, size_clamped * max_grain_size);
    float read_extent = effective_samples * 2.0f * 1.25f;
    int safe = static_cast<int>(read_extent) + SAFE_OFFSET_WARMUP;
    if (safe < SAFE_OFFSET_WARMUP) safe = SAFE_OFFSET_WARMUP;
    constexpr int kHalfBuffer = static_cast<int>(GRANULAR_BUFFER_SIZE / 2);
    if (safe > kHalfBuffer) safe = kHalfBuffer;
    if (!has_wrapped_) {
      int max_allowed = static_cast<int>(samples_recorded_) - SAFE_OFFSET_WARMUP;
      if (safe > max_allowed) safe = max_allowed;
      if (safe < SAFE_OFFSET_WARMUP) safe = SAFE_OFFSET_WARMUP;
    }
    return safe;
  }

  // --- State Accessors ---
  inline size_t GetSamplesRecorded() const { return samples_recorded_; }
  inline bool HasWrapped() const { return has_wrapped_; }
  inline bool IsWarm() const { return has_wrapped_ || (samples_recorded_ >= WARMUP_THRESHOLD); }
  inline bool IsHeld() const { return is_held_; }
  inline int32_t GetWriteHeadPos() const { return static_cast<int32_t>(write_pos_); }
  inline uint64_t GetTotalWrites() const { return total_writes_; }
  inline float GetSampleRate() const { return sample_rate_; }
  inline const int16_t *Data() const { return line_; }

private:
  static inline int32_t WrapIndex(int32_t i) {
    return (i >= static_cast<int32_t>(GRANULAR_BUFFER_SIZE)) ? i - static_cast<int32_t>(GRANULAR_BUFFER_SIZE) : i;
  }
  static inline size_t NextIndex(size_t i) { return (i + 1 >= GRANULAR_BUFFER_SIZE) ? 0 : i + 1; }

  // Records n samples at p[0], p[-1], ... p[-(n - 1)] (no wrap inside the run)
  template <bool kOverdub>
  static inline void RecordRun(const float *in, size_t n, float &fb, float fb_step, float &dc_x,
                               float &dc_y, float dc_r, int16_t *p) {
    float f = fb, x = dc_x, y = dc_y;
    for (size_t k = 0; k < n; ++k) {
      f += fb_step;
      const float dc_in = BufferDcBlocker::Step(in[k], x, y, dc_r);
      int16_t *slot = p - k;
      float accumulated = dc_in + S16ToFloat(*slot) * f;
      if (kOverdub) {
        accumulated = OverdubLimit(accumulated);
      }
      *slot = FloatToS16(accumulated);
    }
    fb = f;
    dc_x = x;
    dc_y = y;
  }

  int16_t line_[GRANULAR_BUFFER_SIZE];
  size_t write_pos_ = 0;
  BufferDcBlocker dc_blocker_;
  float sample_rate_ = 48000.0f;
  size_t samples_recorded_ = 0;
  uint64_t total_writes_ = 0;
  bool has_wrapped_ = false;
  bool is_held_ = false;
  bool was_held_ = false;
  int resume_counter_ = 0;
  float last_frozen_sample_ = 0.0f;
};

} // namespace landscape

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif // GRANULAR_BUFFER_H
