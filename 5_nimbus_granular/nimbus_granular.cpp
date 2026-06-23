#include "daisysp.h"
#include "dsp/simple_granular_processor.h"
#include "dsp/resources.h"
#include <ccam/hw/estuary.h>
#include <ccam/utils/gubbins.h>
#include <cmath>

using namespace daisy;
using namespace daisysp;

static ccam::hw::Estuary hw;
static SimpleGranularProcessor granular_processor;

// Buffer for granular processor (using simplified smaller buffers)
FloatFrame input_buffer[64];
FloatFrame output_buffer[64];

static void AudioCallback(daisy::AudioHandle::InputBuffer  in,
                          daisy::AudioHandle::OutputBuffer out,
                          size_t                           size) {
    hw.ProcessAllControls();
    
    // Map CCAM Estuary controls to granular parameters
    Parameters* params = granular_processor.mutable_parameters();
    
    // Knobs (0-7) map to granular parameters
    params->position      = hw.Knob(0);   // 1: Playback position in buffer
    params->size          = hw.Knob(1);   // 2: Grain size
    params->pitch         = hw.Knob(2);   // 3: Pitch shift
    params->density       = hw.Knob(3);   // 4: Grain density/rate
    params->texture       = hw.Knob(4);   // 5: Texture (filter cutoff)
    params->dry_wet       = hw.Knob(5);   // 6: Dry/wet mix
    params->stereo_spread = hw.Knob(6);   // 7: Stereo spread
    params->reverb        = hw.Knob(7);   // 8: Reverb amount
    
    // Switches control playback mode and quality
    // Switch 1: Feedback amount (up=high, center=medium, down=off)
    int feedback_switch = hw.switches[0].Read();
    if (feedback_switch == 2) {        // UP
        params->feedback = 0.7f;
    } else if (feedback_switch == 1) { // CENTER  
        params->feedback = 0.3f;
    } else {                           // DOWN
        params->feedback = 0.0f;
    }
    
    // Switch 2: Freeze mode (up=freeze, down=normal)
    bool freeze_switch = (hw.switches[1].Read() == 2);
    bool freeze_gate = hw.GateIn1().State(); // Gate 1 can also freeze
    params->freeze = freeze_switch || freeze_gate;
    granular_processor.set_freeze(params->freeze);
    
    // Gate 2 can trigger grains manually
    params->trigger = hw.GateIn2().Trig();
    
    // CV inputs (optional modulation)
    float position_cv = hw.cvins[0]->Value(); // CV 1: Position modulation
    float pitch_cv = hw.cvins[1]->Value();    // CV 2: Pitch modulation  
    float density_cv = hw.cvins[2]->Value();  // CV 3: Density modulation
    float texture_cv = hw.cvins[3]->Value();  // CV 4: Texture modulation
    
    // Apply CV modulation (bipolar ±1V)
    params->position = fmaxf(0.0f, fminf(1.0f, params->position + position_cv * 0.3f));
    params->pitch = fmaxf(0.0f, fminf(1.0f, params->pitch + pitch_cv * 0.3f));
    params->density = fmaxf(0.0f, fminf(1.0f, params->density + density_cv * 0.3f));
    params->texture = fmaxf(0.0f, fminf(1.0f, params->texture + texture_cv * 0.3f));
    
    // Process audio in blocks
    for (size_t i = 0; i < size; i++) {
        input_buffer[i].l = IN_L[i];
        input_buffer[i].r = IN_R[i];
    }
    
    granular_processor.Process(input_buffer, output_buffer, size);
    
    for (size_t i = 0; i < size; i++) {
        OUT_L[i] = output_buffer[i].l;
        OUT_R[i] = output_buffer[i].r;
    }
    
    // Output gates based on grain activity or freeze state
    hw.GateOut1().Write(params->freeze);              // Gate out 1: Freeze indicator
    hw.GateOut2().Write(params->trigger);             // Gate out 2: Trigger echo
    
    // CV outputs: send current parameter values
    hw.WriteCvOut(patch_sm::CV_OUT_1, (params->position - 0.5f) * 5.0f);  // Position as CV
    hw.WriteCvOut(patch_sm::CV_OUT_2, (params->pitch - 0.5f) * 5.0f);     // Pitch as CV
    
    // LED feedback - show grain activity and freeze state
    for (int i = 0; i < 8; i++) {
        float brightness = 0.0f;
        
        switch(i) {
            case 0: brightness = params->position; break;
            case 1: brightness = params->size; break;  
            case 2: brightness = params->pitch; break;
            case 3: brightness = params->density; break;
            case 4: brightness = params->texture; break;
            case 5: brightness = params->dry_wet; break;
            case 6: brightness = params->stereo_spread; break;
            case 7: brightness = params->freeze ? 1.0f : params->reverb; break;
        }
        
        hw.leds[i].Set(brightness);
    }
    
    hw.PostProcess();
}

int main() {
    hw.Init();

    float sample_rate = hw.AudioSampleRate();
    
    // Initialize resources (lookup tables)
    InitResources(sample_rate);
    
    // Initialize granular processor
    granular_processor.Init(sample_rate);
    
    // Set initial parameters to sensible defaults
    Parameters* params = granular_processor.mutable_parameters();
    params->position = 0.5f;
    params->size = 0.3f;
    params->pitch = 0.5f;
    params->density = 0.4f;
    params->texture = 0.6f;
    params->dry_wet = 0.7f;
    params->stereo_spread = 0.3f;
    params->feedback = 0.2f;
    params->reverb = 0.1f;
    params->freeze = false;
    
    // Start audio processing
    hw.StartAudio(AudioCallback);
    
    // Main loop
    while(1) {
        // The audio callback handles everything
        // Could add additional processing here if needed
        System::Delay(1);
    }
}