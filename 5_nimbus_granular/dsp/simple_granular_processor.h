#ifndef NIMBUS_DSP_SIMPLE_GRANULAR_PROCESSOR_H_
#define NIMBUS_DSP_SIMPLE_GRANULAR_PROCESSOR_H_

#include "frame.h"
#include "parameters.h"
#include "parameter_interpolator.h"
#include "utils.h"
#include "daisysp.h"
#include <Utility/delayline.h>
#include <Filters/svf.h>

// Simplified granular processor for CCAM Estuary
// Based on Mutable Instruments Clouds but significantly simplified

const int32_t kBufferSize = 32768;  // 32k buffer (~0.68 seconds at 48kHz)
const int32_t kMaxGrains = 16;      // Maximum number of simultaneous grains

struct Grain
{
    bool active;
    int32_t buffer_pos;
    int32_t size;
    int32_t pos;
    float pitch_ratio;
    float gain_l;
    float gain_r;
    float envelope_pos;
    float envelope_inc;
    
    void Reset() {
        active = false;
        buffer_pos = 0;
        size = 0;
        pos = 0;
        pitch_ratio = 1.0f;
        gain_l = 0.5f;
        gain_r = 0.5f;
        envelope_pos = 0.0f;
        envelope_inc = 0.0f;
    }
    
    void Start(int32_t buf_pos, int32_t grain_size, float pitch, float pan) {
        active = true;
        buffer_pos = buf_pos;
        size = grain_size;
        pos = 0;
        pitch_ratio = pitch;
        envelope_pos = 0.0f;
        envelope_inc = 1.0f / static_cast<float>(grain_size);
        
        // Pan control
        gain_l = cosf(pan * 1.57079632f);  // cos(pan * pi/2)
        gain_r = sinf(pan * 1.57079632f);  // sin(pan * pi/2)
    }
    
    float GetEnvelope() {
        // Simple triangular envelope
        float env = envelope_pos < 0.5f ? envelope_pos * 2.0f : 2.0f - envelope_pos * 2.0f;
        return env * env;  // squared for smoother curve
    }
};

class SimpleGranularProcessor
{
public:
    SimpleGranularProcessor() {
        sample_rate_ = 48000.0f;
        write_head_ = 0;
        freeze_ = false;
        for (int i = 0; i < kMaxGrains; i++) {
            grains_[i].Reset();
        }
    }
    
    ~SimpleGranularProcessor() {}

    void Init(float sample_rate) {
        sample_rate_ = sample_rate;
        
        // Initialize buffer to zero
        for (int32_t i = 0; i < kBufferSize; i++) {
            buffer_l_[i] = 0.0f;
            buffer_r_[i] = 0.0f;
        }
        
        // Initialize filters
        filter_l_.Init(sample_rate);
        filter_r_.Init(sample_rate);
        
        delay_line_.Init();
        delay_feedback_ = 0.6f;
        
        write_head_ = 0;
        grain_trigger_phase_ = 0.0f;
        freeze_ = false;
        
        // Reset parameters
        parameters_.position = 0.5f;
        parameters_.size = 0.5f;
        parameters_.pitch = 0.0f;
        parameters_.density = 0.5f;
        parameters_.texture = 0.5f;
        parameters_.dry_wet = 0.5f;
        parameters_.feedback = 0.0f;
        parameters_.reverb = 0.0f;
        parameters_.freeze = false;
    }

    void Process(FloatFrame* input, FloatFrame* output, size_t size);
    
    Parameters* mutable_parameters() { return &parameters_; }
    
    void set_freeze(bool freeze) { freeze_ = freeze; }
    bool frozen() const { return freeze_; }

private:
    void ProcessGrains(FloatFrame* output, size_t size);
    void TriggerGrain();
    int32_t FindFreeGrain();
    
    float sample_rate_;
    
    // Audio buffer
    float buffer_l_[kBufferSize];
    float buffer_r_[kBufferSize];
    int32_t write_head_;
    
    // Grains
    Grain grains_[kMaxGrains];
    float grain_trigger_phase_;
    
    // Effects
    daisysp::Svf filter_l_;
    daisysp::Svf filter_r_;
    // Simple delay line for basic reverb effect
    daisysp::DelayLine<float, 24000> delay_line_;  // ~0.5 seconds at 48kHz
    float delay_feedback_;
    FloatFrame delay_buffer_[64];  // Small buffer for processing
    
    // State
    bool freeze_;
    Parameters parameters_;
    
    // Internal buffers
    FloatFrame grain_output_[kMaxBlockSize];

};

#endif // NIMBUS_DSP_SIMPLE_GRANULAR_PROCESSOR_H_