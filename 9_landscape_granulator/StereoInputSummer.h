#pragma once
#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * Mono input for a guitar on In L alone, or for a source patched into both inputs:
 * In L by itself until In R carries signal, then (L + R) / 2.
 *
 * In R counts as patched while its block peak rises above kSignalThreshold, and as
 * unpatched after kHoldSec without that; the change is crossfaded over kFadeSec.
 * (Deciding per sample on |R| > 1e-5 flipped on the noise floor of an unpatched
 * input about 20,000 times a second, which distorted the guitar.)
 */
class StereoInputSummer {
public:
    static constexpr float kSignalThreshold = 0.002f; // -54 dBFS peak: far above an open input's noise
    static constexpr float kHoldSec         = 3.0f;   // quiet time before In R counts as unpatched
    static constexpr float kFadeSec         = 0.05f;  // crossfade between L and (L + R) / 2

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        quiet_sec_   = kHoldSec; // boot as "In R unpatched"
        mix_         = 0.0f;
        target_      = 0.0f;
        step_        = 1.0f / (kFadeSec * sample_rate);
    }

    /// Once per audio block, before Process(): updates the In R detector.
    void Prepare(const float *in_r, size_t size) {
        float peak = 0.0f;
        for (size_t i = 0; i < size; i++) peak = fmaxf(peak, fabsf(in_r[i]));
        if (peak > kSignalThreshold) {
            quiet_sec_ = 0.0f;
        } else if (quiet_sec_ < kHoldSec) {
            quiet_sec_ += static_cast<float>(size) / sample_rate_;
        }
        target_ = (quiet_sec_ < kHoldSec) ? 1.0f : 0.0f;
    }

    inline float Process(float in_l, float in_r) {
        if (mix_ < target_) {
            mix_ = fminf(target_, mix_ + step_);
        } else if (mix_ > target_) {
            mix_ = fmaxf(target_, mix_ - step_);
        }
        return in_l + mix_ * ((in_l + in_r) * 0.5f - in_l);
    }

    inline void ProcessBlock(const float *in_l, const float *in_r, float *out, size_t n) {
        if (mix_ == target_) {
            if (mix_ == 0.0f) {
                memcpy(out, in_l, n * sizeof(float));
                return;
            }
            const float m = mix_;
            for (size_t i = 0; i < n; i++) {
                const float l = in_l[i];
                out[i] = l + m * ((l + in_r[i]) * 0.5f - l);
            }
            return;
        }
        float m = mix_;
        const float t = target_;
        const float s = step_;
        for (size_t i = 0; i < n; i++) {
            if (m < t) {
                m = fminf(t, m + s);
            } else if (m > t) {
                m = fmaxf(t, m - s);
            }
            const float l = in_l[i];
            out[i] = l + m * ((l + in_r[i]) * 0.5f - l);
        }
        mix_ = m;
    }

    float StereoMix() const { return mix_; }

private:
    float sample_rate_ = 48000.0f;
    float quiet_sec_   = kHoldSec;
    float mix_         = 0.0f;
    float target_      = 0.0f;
    float step_        = 0.0f;
};
