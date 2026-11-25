#pragma once
#ifndef DSY_RAMPSVF_H
#define DSY_RAMPSVF_H

#include <cmath>

namespace daisysp
{
/** Single-pass Chamberlin state-variable filter with per-sample coefficient ramps.

daisysp::Svf runs every sample twice (2x oversampled) so it stays stable up to
fs/4, and computes five outputs. Below ~6 kHz at 48 kHz one pass is stable for
every damping (res 0..1, damping 2..0), so this filter runs one pass, has no
drive term (always linear) and computes only the low and band outputs: about
half the cost of Svf::ProcessLow().

Coefficients ramp per sample across each block: SetTarget() runs once per block
(a sinf, plus three sqrtf when the resonance changes), BeginBlock()
spreads the change over the block and EndBlock() lands exactly on the target.
Setting the coefficients once per block instead is a 48000 / block-size (1 kHz
at 48) modulation of every swept filter.

Resonance uses the same mapping as Svf::SetRes(): damping = 2 * (1 - res^0.25),
so res 0 is Q 0.5 and res 1 is no damping at all.
*/
class RampSvf
{
  public:
    /** Every cutoff up to this is stable at any resonance (worst case res 0).
        Above it SetTarget() keeps the filter stable by limiting the coefficient. */
    static constexpr float kMaxFreqHz = 6000.0f;

    void Init(float sample_rate)
    {
        pi_over_sr_ = 3.14159265358979f / sample_rate;
        nyquist_    = 0.5f * sample_rate;
        low_ = band_ = 0.0f;
        f_ = target_f_ = 0.0f;
        d_ = target_d_ = 2.0f;
        step_f_ = step_d_ = 0.0f;
        res_ = -1.0f; // forces the damping to be computed by the first SetTarget()
        max_f_ = 0.0f;
        primed_ = false;
    }

    /** Sets this block's target cutoff (Hz) and resonance (0..1). The first call
        after Init() jumps straight to the target instead of ramping. */
    void SetTarget(float freq_hz, float res)
    {
        res = std::fmin(std::fmax(res, 0.0f), 1.0f);
        if(res != res_)
        {
            res_ = res;
            // res^0.25 as two sqrts (single instructions on the M7; powf is a long call)
            target_d_ = 2.0f * (1.0f - std::sqrt(std::sqrt(res)));
            // One pass is stable while f^2 + 2 f d < 4; stay 5% inside that
            max_f_ = 0.95f * (std::sqrt(target_d_ * target_d_ + 4.0f) - target_d_);
        }
        // Up to Nyquist (where 2 sin() peaks), then limited to the stable range
        freq_hz   = std::fmin(std::fmax(freq_hz, 0.0f), nyquist_);
        target_f_ = std::fmin(2.0f * std::sin(freq_hz * pi_over_sr_), max_f_);
        if(!primed_)
        {
            primed_ = true;
            f_      = target_f_;
            d_      = target_d_;
        }
    }

    /** Starts this block's ramp toward the target set by SetTarget(). */
    inline void BeginBlock(float inv_samples)
    {
        step_f_ = (target_f_ - f_) * inv_samples;
        step_d_ = (target_d_ - d_) * inv_samples;
    }

    /** Ends the block exactly on the target. */
    inline void EndBlock()
    {
        f_ = target_f_;
        d_ = target_d_;
    }

    /** One sample; returns the lowpass output */
    inline float ProcessLow(float in)
    {
        Process(in);
        return low_;
    }

    /** One sample; returns the bandpass output */
    inline float ProcessBand(float in)
    {
        Process(in);
        return band_;
    }

    /** One sample; read the outputs with Low() and Band() */
    inline void Process(float in)
    {
        f_ += step_f_;
        d_ += step_d_;
        low_ += f_ * band_;
        const float high = in - low_ - d_ * band_;
        band_ += f_ * high;
    }

    inline float Low() const { return low_; }
    inline float Band() const { return band_; }

  private:
    float pi_over_sr_ = 0.0f;
    float nyquist_    = 24000.0f;
    float low_ = 0.0f, band_ = 0.0f;
    float f_ = 0.0f, d_ = 2.0f;
    float target_f_ = 0.0f, target_d_ = 2.0f;
    float step_f_ = 0.0f, step_d_ = 0.0f;
    float res_   = -1.0f;
    float max_f_ = 0.0f;
    bool  primed_ = false;
};

} // namespace daisysp
#endif
