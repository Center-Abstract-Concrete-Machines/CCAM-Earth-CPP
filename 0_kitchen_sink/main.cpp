#include <array>

// Core libraries for Daisy hardware and DSP
#include "daisysp.h"          // DaisySP: Digital Signal Processing library
#include "Utility/dsp.h"      // DSP utilities (mapping functions, etc.)
#include "Effects/reverbsc.h" // Schroeder reverb effect

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

// DSP Objects - these process audio/control signals
daisysp::Oscillator vco;  // VCO: Voltage Controlled Oscillator for audio synthesis
daisysp::Oscillator lfo;  // LFO: Low Frequency Oscillator for modulation/CV output
daisysp::ReverbSc verb;   // Reverb effect processor

// Constant for converting float (0.0-1.0) to 16-bit unsigned integer (0-65535)
// Used for CV output scaling
static constexpr float MAX_U16_FLOAT = static_cast<float>(0xFFFF);

// REAL-TIME AUDIO CALLBACK - Called at audio sample rate (typically 48kHz)
// This is where you process audio and read control inputs
static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    
    // READ ALL HARDWARE INPUTS
    // This updates all knobs, CV inputs, buttons, etc.
    // Call this once per audio block to get fresh control values
    hw.ProcessAllControls();
    hw.PostProcess();

    // MAP KNOBS TO CONTROL PARAMETERS
    // hw.knobs[0-7] = physical knobs/potentiometers (0.0 to 1.0 range)
    // daisysp::fmap() scales/maps values to useful parameter ranges
    
    // Knob 0 -> Reverb feedback time (0.3 to 0.99)
    float time_cv = daisysp::fmap(hw.knobs[0]->Value(), 0.3f, 0.99f);
    
    // Knob 1 -> Reverb damping frequency (1kHz to 19kHz, logarithmic scaling)
    // LOG mapping gives more musical control for frequency parameters
    float damp_cv = daisysp::fmap(hw.knobs[1]->Value(), 1000.f, 19000.f, daisysp::Mapping::LOG);

    // UPDATE DSP PARAMETERS with mapped control values
    verb.SetFeedback(time_cv);  // Set reverb time from knob 0
    verb.SetLpFreq(damp_cv);    // Set reverb damping from knob 1

    // AUDIO PROCESSING LOOP - Process each sample in the buffer
    for (size_t i = 0; i < size; i++)
    {
        // Process reverb effect on audio inputs
        // in[0][i], in[1][i] = stereo audio inputs from hardware
        // out[0][i], out[1][i] = stereo audio outputs to hardware
        verb.Process(in[0][i], in[1][i], &out[0][i], &out[1][i]);

        // Generate oscillator voice and mix with reverb output
        // Knob 2 controls oscillator amplitude (with small offset to avoid DC)
        float voice = vco.Process() * (hw.knobs[2]->Value() - 0.05f);
        out[0][i] += voice;  // Add voice to left audio output
        out[1][i] += voice;  // Add voice to right audio output
    }
}

// CV OUTPUT CALLBACK - Called at CV sample rate (typically 1kHz)
// This generates control voltages for external hardware (VCOs, VCFs, etc.)
static void CVOutCallback(uint16_t **out, size_t size)
{
    // Process each CV output sample in the buffer
    for(size_t i = 0; i < size; i++)
    {
        // READING CV INPUTS
        // Sum all CV input voltages (hw.cvins = array of CV input objects)
        // Each CV input typically ranges 0-10V, converted to 0.0-1.0 float
        float cvsum = 0.0f;
        for (auto* cvin : hw.cvins) { 
            cvsum += cvin->Value();  // Add each CV input value
        }
        
        // GENERATING CV OUTPUTS
        // out[0], out[1], etc. = CV output channels (depends on hardware)
        // Values are 12-bit (0-4095), right-shifted from 16-bit
        
        // CV OUT 1: Sum of all CV inputs (voltage processor/mixer)
        out[1][i] = static_cast<uint16_t>(cvsum * MAX_U16_FLOAT) >> 4;
        
        // CV OUT 0: LFO output for modulation
        // LFO generates -1 to +1, needs to be scaled to 0-4095 range
        out[0][i] = static_cast<uint16_t>(lfo.Process()) >> 4;
    }
}

int main(void)
{
    // HARDWARE INITIALIZATION
    hw.Init();                        // Initialize all hardware I/O (ADCs, DACs, GPIO, etc.)
    hw.StartAudio(AudioCallback);     // Start audio engine with our callback function
    hw.StartCV(CVOutCallback);        // Start CV output engine with our CV callback

    // DSP OBJECT INITIALIZATION
    // Initialize VCO at audio sample rate (typically 48kHz)
    vco.Init(hw.som.AudioSampleRate());
    vco.SetFreq(440.0f);              // Set VCO frequency to 440Hz (A4)

    // Initialize LFO at CV sample rate (typically 1kHz) 
    lfo.Init(hw.CvOutSampleRate());    
    lfo.SetFreq(220.0f);              // Set LFO frequency to 220Hz
    lfo.SetAmp(MAX_U16_FLOAT);        // Set LFO amplitude to full range for CV output

    // Initialize reverb at audio sample rate
    verb.Init(hw.som.AudioSampleRate());

    // DEBUG/LOGGING SETUP - Commented out to avoid blocking
    // hw.som.StartLog(false);           // Enable logging (false = don't wait for connection)
    // hw.som.PrintLine("Hello world");  // Send debug message

    // MAIN LOOP - Non-real-time processing
    // Real-time audio happens in callbacks, this loop handles:
    // - LED updates, display updates, slow control changes
    // - User interface, preset loading, etc.
    while(1) {
        
        // LED CONTROL EXAMPLE
        // Map knob values directly to LED brightness
        // hw.knobs[i] = knob input (0.0-1.0)
        // hw.leds[i] = LED output (0.0-1.0, where 1.0 = full brightness)
        for (unsigned i = 0; i < hw.knobs.size(); i++) {
            hw.leds[i].Set(hw.knobs[i]->Value());  // LED mirrors knob position
        }

#if __CCAM_TARGET_PLATFORM__ == PLATFORM_EARTH
        // PLATFORM-SPECIFIC CODE
        // Earth platform has fewer knobs, so turn on unused LEDs
        hw.leds[6].Set(1.0f);    // Turn on LED 6 (full brightness)
        hw.leds[7].Set(1.0f);    // Turn on LED 7 (full brightness)

        // BUTTON/GATE INPUT PROCESSING
        // hw.buttons[i] can be physical buttons OR gate/trigger inputs
        // Pressed() returns true for button press or gate trigger
        for (unsigned i = 0; i < hw.leds.size(); i++) {
            if (hw.buttons[i].Pressed()) {
                hw.leds[i].Set(0.0f);    // Turn off LED when button/gate triggered
            }
        }
#endif

        // Control loop timing - run at ~1kHz
        // Most UI updates don't need to be faster than this
        daisy::System::Delay(1);  // 1ms delay
    }
}
