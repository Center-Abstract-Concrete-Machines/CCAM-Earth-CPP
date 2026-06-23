#include "simple_granular_processor.h"
#include <algorithm>
#include <cmath>

void SimpleGranularProcessor::Process(FloatFrame* input, FloatFrame* output, size_t size)
{
    // Write input to buffer unless frozen
    for (size_t i = 0; i < size; i++) {
        if (!freeze_) {
            buffer_l_[write_head_] = input[i].l;
            buffer_r_[write_head_] = input[i].r;
            write_head_ = (write_head_ + 1) % kBufferSize;
        }
        
        // Initialize output
        output[i].l = 0.0f;
        output[i].r = 0.0f;
    }
    
    // Process grains
    ProcessGrains(output, size);
    
    // Apply filtering based on texture parameter
    float cutoff_freq = 200.0f + parameters_.texture * 8000.0f;
    filter_l_.SetFreq(cutoff_freq);
    filter_l_.SetRes(0.7f);
    filter_r_.SetFreq(cutoff_freq);
    filter_r_.SetRes(0.7f);
    
    for (size_t i = 0; i < size; i++) {
        filter_l_.Process(output[i].l);
        filter_r_.Process(output[i].r);
        output[i].l = filter_l_.Low();
        output[i].r = filter_r_.Low();
    }
    
    // Apply simple delay-based reverb
    if (parameters_.reverb > 0.01f) {
        delay_feedback_ = 0.6f + parameters_.reverb * 0.3f;
        
        for (size_t i = 0; i < size; i++) {
            // Simple delay for reverb effect
            float delay_time = 0.3f;  // 300ms delay
            float delayed_l = delay_line_.Read(delay_time * sample_rate_);
            float delayed_r = delay_line_.Read(delay_time * sample_rate_ + 100.0f); // Slight offset for stereo
            
            // Mix input with feedback
            float input_l = output[i].l + delayed_l * delay_feedback_;
            float input_r = output[i].r + delayed_r * delay_feedback_;
            
            // Write to delay line
            delay_line_.Write((input_l + input_r) * 0.5f);
            
            // Mix dry and wet
            output[i].l = output[i].l * (1.0f - parameters_.reverb) + delayed_l * parameters_.reverb;
            output[i].r = output[i].r * (1.0f - parameters_.reverb) + delayed_r * parameters_.reverb;
        }
    }
    
    // Apply dry/wet mix
    for (size_t i = 0; i < size; i++) {
        float dry = 1.0f - parameters_.dry_wet;
        float wet = parameters_.dry_wet;
        output[i].l = input[i].l * dry + output[i].l * wet;
        output[i].r = input[i].r * dry + output[i].r * wet;
    }
}

void SimpleGranularProcessor::ProcessGrains(FloatFrame* output, size_t size)
{
    // Calculate grain trigger rate based on density
    float grain_rate = 0.5f + parameters_.density * 8.0f; // 0.5 to 8.5 grains per second
    float grain_inc = grain_rate / sample_rate_;
    
    for (size_t i = 0; i < size; i++) {
        // Trigger new grains
        grain_trigger_phase_ += grain_inc;
        if (grain_trigger_phase_ >= 1.0f) {
            grain_trigger_phase_ -= 1.0f;
            TriggerGrain();
        }
        
        // Process existing grains
        for (int g = 0; g < kMaxGrains; g++) {
            Grain* grain = &grains_[g];
            if (!grain->active) continue;
            
            // Get buffer position with position offset
            float position_offset = (parameters_.position - 0.5f) * kBufferSize * 0.8f;
            int32_t read_pos = static_cast<int32_t>(grain->buffer_pos + position_offset);
            read_pos = (read_pos + kBufferSize) % kBufferSize;
            
            // Read from buffer with pitch shifting
            float fractional_pos = grain->pos * grain->pitch_ratio;
            int32_t int_pos = static_cast<int32_t>(fractional_pos);
            float frac = fractional_pos - int_pos;
            
            int32_t read_idx1 = (read_pos + int_pos) % kBufferSize;
            int32_t read_idx2 = (read_pos + int_pos + 1) % kBufferSize;
            
            // Linear interpolation
            float sample_l = buffer_l_[read_idx1] + frac * (buffer_l_[read_idx2] - buffer_l_[read_idx1]);
            float sample_r = buffer_r_[read_idx1] + frac * (buffer_r_[read_idx2] - buffer_r_[read_idx1]);
            
            // Apply grain envelope
            float env = grain->GetEnvelope();
            sample_l *= env;
            sample_r *= env;
            
            // Apply stereo spread
            float spread = parameters_.stereo_spread;
            float spread_l = sample_l * grain->gain_l + sample_r * grain->gain_r * spread;
            float spread_r = sample_r * grain->gain_r + sample_l * grain->gain_l * spread;
            
            output[i].l += spread_l * 0.3f; // Scale down to prevent clipping
            output[i].r += spread_r * 0.3f;
            
            // Advance grain
            grain->pos++;
            grain->envelope_pos += grain->envelope_inc;
            
            // Check if grain is finished
            if (grain->pos >= grain->size || grain->envelope_pos >= 1.0f) {
                grain->Reset();
            }
        }
    }
}

void SimpleGranularProcessor::TriggerGrain()
{
    int32_t grain_idx = FindFreeGrain();
    if (grain_idx < 0) return; // No free grains
    
    // Calculate grain parameters
    float grain_size_ms = 10.0f + parameters_.size * 200.0f; // 10ms to 210ms
    int32_t grain_size = static_cast<int32_t>(grain_size_ms * sample_rate_ / 1000.0f);
    
    // Random position around current write head
    float position_var = (Random() - 0.5f) * 0.2f * kBufferSize; // ±10% variation
    int32_t buffer_pos = write_head_ + static_cast<int32_t>(position_var);
    buffer_pos = (buffer_pos + kBufferSize) % kBufferSize;
    
    // Pitch ratio from pitch parameter
    float pitch_ratio = SemitonesToRatio(parameters_.pitch * 24.0f - 12.0f); // ±12 semitones
    
    // Random pan position
    float pan = Random();
    
    grains_[grain_idx].Start(buffer_pos, grain_size, pitch_ratio, pan);
}

int32_t SimpleGranularProcessor::FindFreeGrain()
{
    for (int32_t i = 0; i < kMaxGrains; i++) {
        if (!grains_[i].active) {
            return i;
        }
    }
    return -1; // No free grain found
}