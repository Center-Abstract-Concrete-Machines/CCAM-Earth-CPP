#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY
#include <ccam/hw/estuary.h>
#include <ccam/utils/gubbins.h>

#include "daisysp.h"
#include <array>
#include <cmath>

using namespace daisy;
using namespace daisysp;

namespace {

ccam::hw::Estuary hw;

struct PMParams {
    float morph;
    float decay;
    float tone;
    float body;
    float glitch;
    float drive;
    float accent;
};

class HyperPopPMVoice {
  public:
    void Init(float sample_rate)
    {
        sr_ = sample_rate;
        noise_.Init();
        tilt_.Init(sr_);
        tilt_.SetRes(0.35f);
        Reset();
    }

    void Reset()
    {
        y1_.fill(0.0f);
        y2_.fill(0.0f);
        a1_.fill(0.0f);
        a2_.fill(0.0f);
        gain_.fill(0.0f);

        exciter_env_ = 0.0f;
        click_env_   = 0.0f;
        exciter_dec_ = 0.9995f;
        click_dec_   = 0.995f;

        hold_counter_ = 0;
        hold_period_  = 1;
        held_         = 0.0f;
        quant_steps_  = 1024.0f;
        stereo_sign_  = 1.0f;
    }

    void Choke(float amt)
    {
        amt = fclamp(amt, 0.0f, 1.0f);
        float keep = 1.0f - (0.95f * amt);
        exciter_env_ *= keep;
        click_env_ *= keep;
        for(size_t i = 0; i < kModes; i++)
        {
            y1_[i] *= keep;
            y2_[i] *= keep;
        }
    }

    float Process(const PMParams &p, bool trigger, float audio_exciter, bool use_audio_exciter)
    {
        if(trigger)
            Trigger(p);

        float n = noise_.Process();
        float source = n;
        if(use_audio_exciter)
        {
            // Keep a little noise mixed in so silent or low-level slices still excite the resonator.
            source = (0.9f * tanhf(audio_exciter * 2.0f)) + (0.1f * n);
        }

        float tone_hz = fmap(fclamp(p.tone, 0.0f, 1.0f), 450.0f, 15000.0f, Mapping::LOG);
        tilt_.SetFreq(tone_hz);

        tilt_.Process(source);

        float bright  = tilt_.High();
        float broad   = tilt_.Band();
        float exciter = ((bright * (0.65f + 0.35f * p.tone)) + (broad * (1.0f - p.tone))) * exciter_env_;

        exciter_env_ *= exciter_dec_;
        click_env_ *= click_dec_;

        float click = (source > 0.0f ? 1.0f : -1.0f) * click_env_;
        float input = exciter * noise_mix_ + click * (1.0f - noise_mix_);

        float ring = 0.0f;
        for(size_t i = 0; i < kModes; i++)
        {
            float y = (input * gain_[i]) + (a1_[i] * y1_[i]) + (a2_[i] * y2_[i]);
            y2_[i]  = y1_[i];
            y1_[i]  = y;
            ring += y;
        }

        float low_body = y1_[0] * (0.35f + 0.65f * p.body);
        float v        = (ring * ring_mix_) + (input * (1.0f - ring_mix_) * 0.35f) + low_body * 0.5f;

        float drive = 1.0f + (p.drive * 6.0f) + (p.accent * 2.0f) + (p.body * 1.5f);
        v           = tanhf(v * drive);

        if(p.glitch > 0.02f)
        {
            if(hold_counter_ <= 0)
            {
                held_        = v;
                hold_counter_ = hold_period_;
            }
            hold_counter_--;

            float held_q = roundf(held_ * quant_steps_) / quant_steps_;
            v            = ((1.0f - p.glitch) * v) + (p.glitch * held_q);
        }

        last_level_ = fabsf(v);
        return v;
    }

    float StereoSign() const { return stereo_sign_; }
    float LastLevel() const { return last_level_; }

  private:
    static constexpr size_t kModes = 4;

    void Trigger(const PMParams &p)
    {
        float m = fclamp(p.morph, 0.0f, 1.0f);

        const float hat_freq[kModes]   = {4200.0f, 7000.0f, 9800.0f, 13200.0f};
        const float snr_freq[kModes]   = {190.0f, 320.0f, 2800.0f, 6400.0f};
        const float hat_tau[kModes]    = {0.018f, 0.012f, 0.009f, 0.007f};
        const float snr_tau[kModes]    = {0.32f, 0.22f, 0.08f, 0.05f};
        const float hat_gain[kModes]   = {0.05f, 0.25f, 0.55f, 0.8f};
        const float snr_gain[kModes]   = {0.95f, 0.8f, 0.35f, 0.18f};

        for(size_t i = 0; i < kModes; i++)
        {
            float f = (hat_freq[i] * (1.0f - m)) + (snr_freq[i] * m);
            float t = (hat_tau[i] * (1.0f - m)) + (snr_tau[i] * m);

            // Decay and accent stretch resonator ring time.
            t *= (0.4f + (1.4f * p.decay));
            t *= (0.9f + (0.35f * p.accent));

            float jitter = (noise_.Process() * 0.03f) * (0.2f + p.glitch);
            f *= (1.0f + jitter);
            f = fclamp(f, 40.0f, sr_ * 0.45f);

            float r  = expf(-1.0f / (t * sr_));
            float w  = TWOPI_F * (f / sr_);
            a1_[i]   = 2.0f * r * cosf(w);
            a2_[i]   = -(r * r);
            gain_[i] = (hat_gain[i] * (1.0f - m)) + (snr_gain[i] * m);
        }

        float burst_t = (0.0015f * (1.0f - m)) + (0.02f * m);
        burst_t *= (0.65f + 1.4f * p.decay);

        exciter_env_ = 1.0f + (0.65f * p.accent);
        click_env_   = 0.75f + (0.45f * (1.0f - m)) + (0.25f * p.glitch);

        exciter_dec_ = expf(-1.0f / (sr_ * burst_t));
        click_dec_   = expf(-1.0f / (sr_ * (0.0008f + 0.0035f * (1.0f - m))));

        noise_mix_ = fclamp(0.88f - (0.35f * m) + (0.08f * p.tone), 0.2f, 0.95f);
        ring_mix_  = fclamp(0.35f + (0.6f * m) + (0.15f * p.body), 0.2f, 0.98f);

        hold_period_ = 1 + static_cast<int>(p.glitch * 18.0f);
        float bits   = fmap(1.0f - p.glitch, 4.0f, 11.0f);
        quant_steps_ = powf(2.0f, bits);

        stereo_sign_ = (noise_.Process() > 0.0f) ? 1.0f : -1.0f;
    }

    float sr_ = 48000.0f;

    WhiteNoise noise_;
    Svf        tilt_;

    std::array<float, kModes> y1_;
    std::array<float, kModes> y2_;
    std::array<float, kModes> a1_;
    std::array<float, kModes> a2_;
    std::array<float, kModes> gain_;

    float exciter_env_ = 0.0f;
    float click_env_   = 0.0f;
    float exciter_dec_ = 0.9995f;
    float click_dec_   = 0.995f;

    float noise_mix_ = 0.85f;
    float ring_mix_  = 0.4f;

    int   hold_counter_ = 0;
    int   hold_period_  = 1;
    float held_         = 0.0f;
    float quant_steps_  = 1024.0f;

    float stereo_sign_ = 1.0f;
    float last_level_  = 0.0f;
};

HyperPopPMVoice voice;

float sample_rate      = 48000.0f;
int   gate1_pulse_left = 0;
int   gate2_pulse_left = 0;

float env_follow = 0.0f;
float audio_presence = 0.0f;

static constexpr int GATE_PULSE_SAMPLES = 220;

PMParams MakeParams(float accent, bool snare_bias)
{
    float morph = hw.Knob(0) + (hw.cvins[0]->Value() * 0.55f);
    float decay = hw.Knob(1) + (hw.cvins[1]->Value() * 0.45f);
    float glitch = hw.Knob(2) + (hw.cvins[3]->Value() * 0.55f);
    float body = hw.Knob(3);
    float tone = hw.Knob(4) + (hw.cvins[2]->Value() * 0.45f);
    float drive = hw.Knob(5);

    int morph_switch = hw.switches[0].Read();
    if(morph_switch == Switch3::POS_LEFT)
        morph *= 0.55f;
    else if(morph_switch == Switch3::POS_RIGHT)
        morph = 0.45f + (morph * 0.55f);

    if(snare_bias)
        morph = fclamp(morph + 0.2f, 0.0f, 1.0f);

    PMParams p;
    p.morph  = fclamp(morph, 0.0f, 1.0f);
    p.decay  = fclamp(decay, 0.0f, 1.0f);
    p.glitch = fclamp(glitch, 0.0f, 1.0f);
    p.body   = fclamp(body, 0.0f, 1.0f);
    p.tone   = fclamp(tone, 0.0f, 1.0f);
    p.drive  = fclamp(drive, 0.0f, 1.0f);
    p.accent = fclamp(accent, 0.0f, 1.0f);
    return p;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
    hw.ProcessAllControls();

    bool trig1  = hw.GateIn1().Trig();
    bool trig2  = hw.GateIn2().Trig();
    bool gate2h = hw.GateIn2().State();

    int trig_mode = hw.switches[1].Read();

    float spread = hw.Knob(6);
    float level  = hw.Knob(7);

    bool do_trigger = false;
    bool accent_hit = false;
    bool snare_bias = false;

    if(trig_mode == Switch3::POS_LEFT)
    {
        // RS-9 friendly: gate 1 clocks main voice, gate 2 adds accent/open behavior.
        do_trigger = trig1 || trig2;
        accent_hit = trig2;
        snare_bias = trig2;
    }
    else if(trig_mode == Switch3::POS_CENTER)
    {
        do_trigger = trig1;
        accent_hit = false;
        snare_bias = false;
        if(gate2h)
            voice.Choke(0.9f);
    }
    else
    {
        do_trigger = trig1 || trig2;
        accent_hit = trig2;
        snare_bias = trig2;
    }

    PMParams p = MakeParams(accent_hit ? 1.0f : 0.0f, snare_bias);

    if(do_trigger)
    {
        gate1_pulse_left = GATE_PULSE_SAMPLES;
        if(accent_hit)
            gate2_pulse_left = GATE_PULSE_SAMPLES;
    }

    for(size_t i = 0; i < size; i++)
    {
        bool fire = (i == 0) && do_trigger;

        float mono_in = 0.5f * (IN_L[i] + IN_R[i]);
        audio_presence = (0.999f * audio_presence) + (0.001f * fabsf(mono_in));
        bool use_audio_exciter = audio_presence > 0.0025f;

        float v = voice.Process(p, fire, mono_in, use_audio_exciter);

        float width  = (voice.StereoSign() * spread * 0.35f);
        float out_l  = v * (1.0f - width);
        float out_r  = v * (1.0f + width);
        float dry_in = 0.15f;

        OUT_L[i] = (out_l * level) + (IN_L[i] * dry_in);
        OUT_R[i] = (out_r * level) + (IN_R[i] * dry_in);

        env_follow = (0.995f * env_follow) + (0.005f * voice.LastLevel());

        if(gate1_pulse_left > 0)
            gate1_pulse_left--;
        if(gate2_pulse_left > 0)
            gate2_pulse_left--;

        hw.GateOut1().Write(gate1_pulse_left > 0);
        hw.GateOut2().Write(gate2_pulse_left > 0);

        hw.WriteCvOut(daisy::patch_sm::CV_OUT_1, fclamp(env_follow * 5.0f, 0.0f, 5.0f));
        hw.WriteCvOut(daisy::patch_sm::CV_OUT_2, p.morph * 5.0f);
    }

    hw.leds[0].Set(p.morph);
    hw.leds[1].Set(p.decay);
    hw.leds[2].Set(p.glitch);
    hw.leds[3].Set(p.body);
    hw.leds[4].Set(p.tone);
    hw.leds[5].Set(p.drive);
    hw.leds[6].Set((gate1_pulse_left > 0) ? 1.0f : 0.0f);
    hw.leds[7].Set((gate2_pulse_left > 0) ? 1.0f : 0.08f);

    hw.PostProcess();
}

} // namespace

int main(void)
{
    hw.Init();
    for(size_t i = 0; i < 12; i++)
    {
        System::Delay(1);
        hw.ProcessAllControls();
    }

    sample_rate = hw.AudioSampleRate();
    voice.Init(sample_rate);

    hw.StartAudio(AudioCallback);

    while(1)
    {
        System::Delay(100);
    }
}
