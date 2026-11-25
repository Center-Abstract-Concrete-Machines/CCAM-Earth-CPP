#pragma once
#ifndef DIFFUSER_H
#define DIFFUSER_H

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("O3")
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Utility/dsp.h"

/**
 * AllpassStage
 * Delay-line allpass (w = x + g * d; y = d - g * w) on a caller-owned buffer, with a
 * coefficient that can ramp linearly over a number of samples (normally one block, so
 * a block-rate change never steps: no 1 kHz sideband whine).
 *
 * ProcessBlock() keeps the coefficient, ramp and pointer in registers and splits the
 * block only where the circular buffer wraps, ramping or not. Its output is identical,
 * bit for bit, to calling Process() once per sample.
 */
class AllpassStage {
public:
  void Reset() {
    ptr_ = 0;
    ramp_left_ = 0;
    if (buf_ && len_ > 0) {
      std::memset(buf_, 0, len_ * sizeof(float));
    }
  }

  inline float Process(float in) {
    if (!buf_ || len_ == 0) {
      return in;
    }
    if (ramp_left_ > 0) {
      coef_ += step_;
      if (--ramp_left_ == 0) {
        coef_ = target_;
      }
    }
    const float buf_out = buf_[ptr_];
    const float w = in + coef_ * buf_out;
    buf_[ptr_] = w;
    if (++ptr_ >= len_) {
      ptr_ = 0;
    }
    return -coef_ * w + buf_out;
  }

  /**
   * Runs stages[0..N-1] in series over a block in place, in one pass: the data is loaded and
   * stored once per sample instead of once per stage. Each stage's output is identical to its
   * own ProcessBlock(). The fused pass handles stages that are steady or ramp over exactly this
   * block (what the engine's per-block setters do); anything else runs stage by stage.
   */
  template <size_t N, typename Stage>
  static void ProcessChain(Stage *stages, float *data, size_t n) {
    if (n == 0) {
      return;
    }
    AllpassStage *chain[N];
    bool ramping = false;
    for (size_t i = 0; i < N; i++) {
      chain[i] = &stages[i];
      const AllpassStage &st = *chain[i];
      if (!st.buf_ || st.len_ == 0 || (st.ramp_left_ != 0 && st.ramp_left_ != n)) {
        for (size_t j = 0; j < N; j++) {
          stages[j].ProcessBlock(data, n);
        }
        return;
      }
      ramping = ramping || (st.ramp_left_ != 0);
    }
    if (ramping) {
      ChainRun<N, true>(chain, data, n);
    } else {
      ChainRun<N, false>(chain, data, n);
    }
  }

  /** Processes a block in place. */
  inline void ProcessBlock(float *data, size_t n) {
    if (!buf_ || len_ == 0 || n == 0) {
      return;
    }
    if (ramp_left_ == 0) {
      Run<false>(data, n, 0.0f);
    } else if (ramp_left_ <= n) {
      // The ramp ends in this block: its last sample uses exactly the target
      const size_t ramped = ramp_left_ - 1;
      Run<true>(data, ramped, step_);
      coef_ = target_;
      ramp_left_ = 0;
      Run<false>(data + ramped, n - ramped, 0.0f);
    } else {
      Run<true>(data, n, step_);
      ramp_left_ -= n;
    }
  }

protected:
  void Attach(float *buffer, size_t length, float coef) {
    buf_ = buffer;
    len_ = length;
    coef_ = target_ = coef;
    step_ = 0.0f;
    Reset();
  }

  // Moves the coefficient to target, linearly over ramp_samples (0 = immediately)
  void RampTo(float target, size_t ramp_samples) {
    target_ = target;
    if (target_ == coef_) {
      ramp_left_ = 0;
    } else if (ramp_samples == 0) {
      coef_ = target_;
      ramp_left_ = 0;
    } else {
      step_ = (target_ - coef_) / static_cast<float>(ramp_samples);
      ramp_left_ = ramp_samples;
    }
  }

  float *buf_ = nullptr;
  size_t len_ = 0;
  float coef_ = 0.0f;
  float target_ = 0.0f;
  float step_ = 0.0f;
  size_t ramp_left_ = 0;
  size_t ptr_ = 0;

private:
  // One sample through one stage at buffer slot b
  static inline float Tick(float x, float c, float *b) {
    const float buf_out = *b;
    const float w = x + c * buf_out;
    *b = w;
    return -c * w + buf_out;
  }

  // Fused series pass. With kRamp, ramping stages step their coefficient on every sample but
  // the last, which uses the exact target (steady stages step by 0 and their target is coef_).
  template <size_t N, bool kRamp>
  static void ChainRun(AllpassStage *const *chain, float *__restrict data, size_t n) {
    float *buf[N];
    size_t len[N], ptr[N];
    float c[N], step[N];
    for (size_t i = 0; i < N; i++) {
      buf[i] = chain[i]->buf_;
      len[i] = chain[i]->len_;
      ptr[i] = chain[i]->ptr_;
      c[i] = chain[i]->coef_;
      step[i] = (chain[i]->ramp_left_ != 0) ? chain[i]->step_ : 0.0f;
    }
    const size_t body = kRamp ? n - 1 : n;
    size_t k = 0;
    while (k < body) {
      // Longest run on which no stage's pointer wraps
      size_t chunk = body - k;
      float *p[N];
      for (size_t i = 0; i < N; i++) {
        chunk = std::min(chunk, len[i] - ptr[i]);
        p[i] = buf[i] + ptr[i];
      }
      for (size_t j = 0; j < chunk; j++) {
        float x = data[k + j];
        for (size_t i = 0; i < N; i++) {
          if (kRamp) {
            c[i] += step[i];
          }
          x = Tick(x, c[i], p[i] + j);
        }
        data[k + j] = x;
      }
      for (size_t i = 0; i < N; i++) {
        ptr[i] += chunk;
        if (ptr[i] >= len[i]) {
          ptr[i] = 0;
        }
      }
      k += chunk;
    }
    if (kRamp) {
      float x = data[n - 1];
      for (size_t i = 0; i < N; i++) {
        c[i] = chain[i]->target_;
        x = Tick(x, c[i], buf[i] + ptr[i]);
        if (++ptr[i] >= len[i]) {
          ptr[i] = 0;
        }
      }
      data[n - 1] = x;
    }
    for (size_t i = 0; i < N; i++) {
      chain[i]->ptr_ = ptr[i];
      chain[i]->coef_ = c[i];
      chain[i]->ramp_left_ = 0;
    }
  }

  template <bool kRamp>
  inline void Run(float *__restrict data, size_t n, float step) {
    float *__restrict const buf = buf_;
    const size_t len = len_;
    size_t ptr = ptr_;
    float c = coef_;
    while (n > 0) {
      const size_t avail = len - ptr;
      const size_t chunk = (n < avail) ? n : avail;
      float *__restrict b = buf + ptr;
      for (size_t k = 0; k < chunk; ++k) {
        if (kRamp) {
          c += step;
        }
        const float buf_out = b[k];
        const float w = data[k] + c * buf_out;
        b[k] = w;
        data[k] = -c * w + buf_out;
      }
      ptr += chunk;
      if (ptr >= len) {
        ptr = 0;
      }
      data += chunk;
      n -= chunk;
    }
    ptr_ = ptr;
    coef_ = c;
  }
};

/**
 * SchroederDiffuser
 * Classic Schroeder allpass diffuser. Prime buffer lengths smear transients without
 * periodic resonant coloration.
 */
class SchroederDiffuser : public AllpassStage {
public:
  void Init(float *buffer, size_t size, float feedback = 0.6f) { Attach(buffer, size, feedback); }

  // Feedback is clamped to [-0.80, +0.80] to guarantee unconditional stability
  inline void SetFeedback(float g, size_t ramp_samples = 0) {
    RampTo(std::fmax(-0.80f, std::fmin(0.80f, g)), ramp_samples);
  }

  inline float GetFeedback() const { return coef_; }
  inline size_t GetSize() const { return len_; }
};

/**
 * CloudAllpass
 * Allpass for dense reverb washes: the coefficient follows a reverb time,
 * coef = exp(-6.9078 * loop_time / rev_time) (-60 dB after rev_time), capped at 0.95.
 */
class CloudAllpass : public AllpassStage {
public:
  // Loop length = delay_sec * sample_rate, capped to the buffer's max_size samples
  void Init(float *buffer, size_t max_size, float delay_sec, float sample_rate) {
    size_t loop = static_cast<size_t>(delay_sec * sample_rate + 0.5f);
    if (loop > max_size) {
      loop = max_size;
    } else if (loop < 4) {
      loop = 4;
    }
    loop_time_ = static_cast<float>(loop) / sample_rate;
    rev_time_ = -1.0f;
    Attach(buffer, loop, 0.0f);
  }

  inline void SetRevTime(float rev_time, size_t ramp_samples = 0) {
    rev_time = std::fmax(rev_time, 0.05f);
    if (rev_time != rev_time_) {
      rev_time_ = rev_time;
      // Hard stability ceiling: feedback never reaches unity (0.95 max)
      target_ = std::fmin(0.95f, daisysp::pow10f(-3.0f * loop_time_ / rev_time_));
    }
    RampTo(target_, ramp_samples);
  }

  inline float GetCoef() const { return coef_; }
  inline size_t GetLoopSamples() const { return len_; }

private:
  float loop_time_ = 0.01f;
  float rev_time_ = -1.0f;
};

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

#endif // DIFFUSER_H
