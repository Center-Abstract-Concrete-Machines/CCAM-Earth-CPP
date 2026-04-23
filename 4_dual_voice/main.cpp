#include <array>

// Core libraries for Daisy hardware and DSP
#include "daisysp.h"          // DaisySP: Digital Signal Processing library
#include "Utility/dsp.h"      // DSP utilities (mapping functions, etc.)
#include "Filters/svf.h"      // State Variable Filter
#include "Control/adsr.h"     // ADSR envelope with segment constants

// Platform definitions - allows same code to work on different hardware
#define PLATFORM_ESTUARY 1
#define PLATFORM_EARTH 2

#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY

// Hardware abstraction: Different platforms have different I/O configurations
// - Estuary: More knobs, CV ins/outs, buttons, LEDs
// - Earth: Fewer knobs, different I/O layout
#if __CCAM_TARGET_PLATFORM__ == PLATFORM_ESTUARY
    #include <ccam/hw/estuary.h>
    ccam::hw::Estuary hw;  // Hardware interface object - provides access to all I/O
#elif __CCAM_TARGET_PLATFORM__ == PLATFORM_EARTH
    #include <ccam/hw/earth.h>
    ccam::hw::Earth hw;    // Different hardware with different I/O capabilities
#else
    #error Unknown target platform please define __CCAM_TARGET_PLATFORM__
#endif

// SINGLE VOICE SYNTHESIZER ARCHITECTURE
// Voice: VCO -> SVF -> VCA (modulated by ADSR Envelope)

// DSP Objects
daisysp::Oscillator vco;       // Variable waveform oscillator
daisysp::Svf        svf;       // State variable filter  
daisysp::Adsr       env;       // ADSR envelope generator

// Shared LED stage state from audio thread:
// 0=off, 1=attack, 2=decay, 3=sustain, 4=release
volatile uint8_t g_env_led_stage = 0;
volatile float   g_decay_time_s  = 0.1f;

// REAL-TIME AUDIO CALLBACK - Single Voice Synthesizer Processing
static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    
    // Process controls at audio rate for proper CV/gate response
    hw.ProcessAllControls();
    
    // Rate limit ADSR parameter updates (every ~1ms instead of every sample)
    static uint32_t adsr_update_counter = 0;
    adsr_update_counter++;
    if(adsr_update_counter >= 48) { // Update every 48 samples (~1ms at 48kHz)
        adsr_update_counter = 0;
        
        // ADSR ENVELOPE CONTROLS - Knobs 1,2,3,4 = Attack, Decay, Sustain, Release
        env.SetAttackTime(daisysp::fmap(hw.knobs[0]->Value(), 0.001f, 3.0f));
        g_decay_time_s = daisysp::fmap(hw.knobs[1]->Value(), 0.001f, 3.0f);
        env.SetDecayTime(g_decay_time_s);
        env.SetSustainLevel(hw.knobs[2]->Value());
        env.SetReleaseTime(daisysp::fmap(hw.knobs[3]->Value(), 0.001f, 5.0f));

        // SVF CONTROLS - Knob 7/8 are primary, CV3/4 are bipolar modulation.
        // Treat CV3/CV4 as modulation in the range +/-5V (from 0..8V input).
        float cv3_mod_volts = (hw.cvins[2]->Process() * 10.0f) - 5.0f; // -5V..+5V
        float cv4_mod_volts = (hw.cvins[3]->Process() * 10.0f) - 5.0f; // -5V..+5V

        // Knobs are primary base controls in 0..1 domain.
        float res_base_norm   = hw.knobs[6]->Value();
        float drive_base_norm = hw.knobs[7]->Value();

        // Convert modulation volts to normalized modulation (+/-0.5 around base).
        float res_mod_norm   = cv3_mod_volts / 10.0f;
        float drive_mod_norm = cv4_mod_volts / 10.0f;

        float res_norm   = res_base_norm + res_mod_norm;
        float drive_norm = drive_base_norm + drive_mod_norm;

        if(res_norm < 0.0f)
            res_norm = 0.0f;
        else if(res_norm > 1.0f)
            res_norm = 1.0f;

        if(drive_norm < 0.0f)
            drive_norm = 0.0f;
        else if(drive_norm > 1.0f)
            drive_norm = 1.0f;

        // SetRes expects 0..1 and SetDrive expects roughly 0..10 before internal scaling.
        float resonance = daisysp::fmap(res_norm, 0.05f, 0.98f);
        float drive     = daisysp::fmap(drive_norm, 10.0f, 0.0f);
        svf.SetRes(resonance);
        svf.SetDrive(drive);
    }
    
    // CV CONTROLS
    // Assumption: incoming CV is 0-8V unipolar where 1V = 1 octave.
    // CV1 -> Pitch (V/Oct), CV2 -> Filter Frequency control
    float cv1_raw   = hw.cvins[0]->Process();
    float cv1_volts = cv1_raw * 8.0f; // 0..8V

    // Knob 5 pitch offset: +/-2V (i.e. +/-2 octaves)
    float pitch_offset_volts = (hw.knobs[4]->Value() - 0.5f) * 4.0f;

    // V/Oct: f = f0 * 2^(V). Use A1 (55Hz) as 0V base.
    float pitch = 55.0f * powf(2.0f, cv1_volts + pitch_offset_volts);

    float cv2_raw   = hw.cvins[1]->Process();
    float cv2_volts = cv2_raw * 8.0f; // 0..8V

    // Knob 6 filter offset: widened to +/-8V around CV2 for a larger sweep.
    float filt_offset_volts = (hw.knobs[5]->Value() - 0.5f) * 16.0f;
    float filt_total_volts  = cv2_volts + filt_offset_volts;

    // Clamp control domain to 0..8V, then map exponentially to 20Hz..18kHz.
    if(filt_total_volts < 0.0f)
        filt_total_volts = 0.0f;
    else if(filt_total_volts > 8.0f)
        filt_total_volts = 8.0f;

    float filt_span_exp = log2f(18000.0f / 20.0f);
    float filt_freq     = 20.0f * powf(2.0f, (filt_total_volts / 8.0f) * filt_span_exp);

    // FILTER MODE SELECTION (3-way switch) 
    int filter_position = hw.switches[0].Read(); // 0=Center, 1=Up, 2=Down

    // WAVEFORM SELECTION (switch 2): Sine / Triangle / Saw
    int waveform_position = hw.switches[1].Read();
    switch(waveform_position)
    {
        case 1: vco.SetWaveform(daisysp::Oscillator::WAVE_SIN); break;
        case 2: vco.SetWaveform(daisysp::Oscillator::WAVE_SAW); break;
        default: vco.SetWaveform(daisysp::Oscillator::WAVE_TRI); break;
    }

    // Update oscillator and filter frequencies
    vco.SetFreq(pitch);
    svf.SetFreq(filt_freq);

    // AUDIO PROCESSING LOOP
    static bool prev_gate_state       = false;
    static uint8_t prev_seg           = daisysp::ADSR_SEG_IDLE;
    static bool decay_timer_armed     = false;
    static int32_t decay_samples_left = 0;
    static float sample_rate          = hw.som.AudioSampleRate();

    for (size_t i = 0; i < size; i++) {
        // READ GATE STATE FOR SUSTAINED ENVELOPE CONTROL
        bool gate_state = hw.gate_inputs[0]->State();

        if(gate_state && !prev_gate_state)
        {
            // New note-on: reset decay timer tracking for this gate cycle.
            decay_timer_armed = false;
            decay_samples_left = 0;
        }
        else if(!gate_state)
        {
            decay_timer_armed = false;
            decay_samples_left = 0;
        }
        
        // SYNTHESIS CHAIN: VCO -> SVF -> VCA
        float osc_out = vco.Process();
        
        // Process through state variable filter
        svf.Process(osc_out);
        float filt_out;
        switch(filter_position) {
            case 1: filt_out = svf.Low(); break;   // LEFT = LP mode
            case 2: filt_out = svf.High(); break;  // RIGHT = HP mode
            default: filt_out = svf.Band(); break; // Center = BP mode (default)
        }
        
        // Apply envelope to create VCA (Voltage Controlled Amplifier)
        float envelope_out = env.Process(gate_state);
        float voice_out = filt_out * envelope_out;

        uint8_t seg = env.GetCurrentSegment();

        // Start decay timer exactly when envelope transitions into DECAY.
        if(seg == daisysp::ADSR_SEG_DECAY && prev_seg != daisysp::ADSR_SEG_DECAY && gate_state)
        {
            decay_timer_armed = true;
            decay_samples_left = static_cast<int32_t>(g_decay_time_s * sample_rate);
            if(decay_samples_left < 0)
                decay_samples_left = 0;
        }

        if(decay_timer_armed && gate_state && decay_samples_left > 0)
            decay_samples_left--;

        uint8_t led_stage = 0;
        if(seg == daisysp::ADSR_SEG_ATTACK)
            led_stage = 1;
        else if(seg == daisysp::ADSR_SEG_RELEASE)
            led_stage = 4;
        else if(gate_state && decay_timer_armed && decay_samples_left <= 0)
            // Sustain: configured decay duration elapsed and gate is still high.
            led_stage = 3;
        else if(seg == daisysp::ADSR_SEG_DECAY)
            led_stage = 2;

        g_env_led_stage = led_stage;
        prev_gate_state = gate_state;
        prev_seg = seg;

        // OUTPUT TO BOTH CHANNELS
        out[0][i] = voice_out;  // Left channel
        out[1][i] = voice_out;  // Right channel
    }
}

int main(void)
{
    // HARDWARE INITIALIZATION
    hw.Init();                        // Initialize all hardware I/O
    hw.StartAudio(AudioCallback);     // Start audio engine

    // DSP INITIALIZATION
    float sample_rate = hw.som.AudioSampleRate();
    
    // Initialize oscillator
    vco.Init(sample_rate);
    vco.SetWaveform(daisysp::Oscillator::WAVE_SAW); // Sawtooth default
    vco.SetAmp(0.7f);  // Set oscillator amplitude
    vco.SetFreq(261.63f);           // Middle C default
    
    // Initialize state variable filter
    svf.Init(sample_rate);
    svf.SetFreq(1000.0f);           // 1kHz default filter frequency
    svf.SetRes(0.7f);               // Set filter resonance
    
    // Initialize ADSR envelope
    env.Init(sample_rate);
    env.SetSustainLevel(0.7f);  // Default sustain level
    env.SetAttackTime(0.01f);   // 10ms attack
    env.SetDecayTime(0.1f);     // 100ms decay
    env.SetReleaseTime(0.5f);   // 500ms release

    // MAIN CONTROL LOOP
    while(1) {
        // Minimal main loop - audio callback handles most processing
        daisy::System::Delay(1);
        
        // Clear all LEDs first
        for(int i = 0; i < 4; i++) {
            hw.leds[i].Set(0.0f);
        }

        switch(g_env_led_stage) {
            case 1: hw.leds[0].Set(1.0f); break; // Attack
            case 2: hw.leds[1].Set(1.0f); break; // Decay
            case 3: hw.leds[2].Set(1.0f); break; // Sustain
            case 4: hw.leds[3].Set(1.0f); break; // Release
            default: break;
        }

        hw.PostProcess();
    }
}