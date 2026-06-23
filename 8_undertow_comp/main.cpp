#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY
#include <ccam/hw/estuary.h>
#include "daisysp.h"
#include "Dynamics/compressor.h"

using namespace daisy;
using namespace daisysp;

static ccam::hw::Estuary hw;

static Compressor comp_m, comp_s;
static Svf        sidechain_hpf;

static float rms_follow = 0.0f;
static float makeup_db  = 0.0f;

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    hw.ProcessAllControls();

    float thresh  = fclamp(-40.0f * hw.Knob(0) + hw.cvins[0]->Value() * 16.0f,
                           -80.0f, 0.0f);
    float ratio_k = fclamp(fmap(hw.Knob(1), 1.0f, 20.0f, Mapping::LOG)
                           + hw.cvins[1]->Value() * 10.0f, 1.0f, 40.0f);
    float attack  = fmap(fclamp(hw.Knob(2) + hw.cvins[2]->Value() * 0.3f, 0.0f, 1.0f),
                         0.0001f, 0.2f, Mapping::LOG);
    float release = fmap(fclamp(hw.Knob(3) + hw.cvins[3]->Value() * 0.3f, 0.0f, 1.0f),
                         0.01f, 2.0f, Mapping::LOG);
    float input_gain = powf(10.0f, (hw.Knob(4) - 0.5f) * 2.0f);
    makeup_db        = hw.Knob(5) * 24.0f;
    float drive      = 1.0f + hw.Knob(6) * 5.0f;
    float drywet     = hw.Knob(7);

    auto sw1 = hw.switches[0].Read();
    auto sw2 = hw.switches[1].Read();

    bool bypass      = hw.GateIn1().State();
    bool duck_active = hw.GateIn2().State();
    bool sc_external = (sw1 == Switch3::POS_RIGHT);
    bool sc_hpf      = (sw1 == Switch3::POS_CENTER);
    bool ms_mid_only = (sw2 == Switch3::POS_CENTER);
    bool ms_side_emph = (sw2 == Switch3::POS_RIGHT);

    float side_ratio = ms_side_emph ? fminf(ratio_k * 1.5f, 40.0f) : ratio_k;

    comp_m.SetThreshold(thresh);
    comp_m.SetRatio(ratio_k);
    comp_m.SetAttack(attack);
    comp_m.SetRelease(release);
    comp_m.SetMakeup(makeup_db);

    comp_s.SetThreshold(thresh);
    comp_s.SetRatio(ms_mid_only ? 1.0f : side_ratio);
    comp_s.SetAttack(attack);
    comp_s.SetRelease(release);
    comp_s.SetMakeup(ms_mid_only ? 0.0f : makeup_db);

    for(size_t i = 0; i < size; i++)
    {
        if(bypass)
        {
            OUT_L[i] = IN_L[i];
            OUT_R[i] = IN_R[i];
            continue;
        }

        float in_l = IN_L[i] * input_gain;
        float in_r = IN_R[i] * input_gain;

        float mid, side, key;

        if(sc_external)
        {
            mid  = in_l;
            side = 0.0f;
            key  = in_r;
        }
        else
        {
            mid  = (in_l + in_r) * 0.5f;
            side = (in_l - in_r) * 0.5f;
            if(sc_hpf)
            {
                sidechain_hpf.Process(mid);
                key = sidechain_hpf.High();
            }
            else
            {
                key = mid;
            }
        }

        if(duck_active)
            key = fmaxf(key, 1.0f);

        // Shared key drives both compressors (stereo-linked detection)
        comp_m.Process(key);
        float comp_mid = comp_m.Apply(mid);

        comp_s.Process(key);
        float comp_side = comp_s.Apply(side);

        // Normalized tanh saturation: linear for small signals, soft clip for large
        comp_mid  = tanhf(comp_mid  * drive) / drive;
        comp_side = tanhf(comp_side * drive) / drive;

        // Parallel compression
        float mid_out  = drywet * comp_mid  + (1.0f - drywet) * mid;
        float side_out = drywet * comp_side + (1.0f - drywet) * side;

        float out_l, out_r;
        if(sc_external)
        {
            out_l = out_r = mid_out;
        }
        else
        {
            out_l = mid_out + side_out;
            out_r = mid_out - side_out;
        }

        OUT_L[i] = out_l;
        OUT_R[i] = out_r;

        rms_follow = 0.999f * rms_follow + 0.001f * fabsf(mid_out);
    }

    // Gain reduction meter: 8 LEDs at 3 dB per step (0–24 dB range)
    // GetGain() returns gain_rec + makeup in dB; subtract makeup to isolate reduction
    float gr_raw = comp_m.GetGain() - makeup_db;
    float gr_db  = fabsf(fminf(gr_raw, 0.0f));
    int   lit    = (int)(gr_db / 3.0f);
    if(lit > 8) lit = 8;
    for(int i = 0; i < 8; i++)
        hw.leds[i].Set(i < lit ? 1.0f : 0.0f);

    hw.WriteCvOut(patch_sm::CV_OUT_1, fclamp(gr_db / 24.0f * 5.0f, 0.0f, 5.0f));
    hw.WriteCvOut(patch_sm::CV_OUT_2, fclamp(rms_follow * 5.0f, 0.0f, 5.0f));

    hw.PostProcess();
}

int main(void)
{
    hw.Init();
    float sr = hw.AudioSampleRate();

    comp_m.Init(sr);
    comp_s.Init(sr);

    sidechain_hpf.Init(sr);
    sidechain_hpf.SetFreq(100.0f);
    sidechain_hpf.SetRes(0.5f);

    hw.StartAudio(AudioCallback);
    while(1) { System::Delay(100); }
}
