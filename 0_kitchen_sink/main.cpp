#include <array>


#include "daisysp.h"
#include "Utility/dsp.h"
#include "Effects/reverbsc.h"

// Compile-time platform IDs.
#define PLATFORM_ESTUARY 1
#define PLATFORM_EARTH 2
// Choose the target platform before compiling.
#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY

#if __CCAM_TARGET_PLATFORM__ == PLATFORM_ESTUARY
    #include <ccam/hw/estuary.h>
    ccam::hw::Estuary hw;
#elif __CCAM_TARGET_PLATFORM__ == PLATFORM_EARTH
    #include <ccam/hw/earth.h>
    ccam::hw::Earth hw;
#else
    #error Unknown target platform please define __CCAM_TARGET_PLATFORM__
#endif

// Global DSP objects that stay alive for the whole program.
daisysp::Oscillator vco;
daisysp::Oscillator lfo;
daisysp::ReverbSc verb;

// Full-scale 16-bit value (65535.0f), used for CV scaling.
static constexpr float MAX_U16_FLOAT = static_cast<float>(0xFFFF);

// Runs on the audio thread for each block of stereo samples.
static void AudioCallback(daisy::AudioHandle::InputBuffer in,
            daisy::AudioHandle::OutputBuffer out, 
            size_t size) {
    // Refresh knobs/CV/buttons and update LED driver state.
    hw.ProcessAllControls();
    hw.PostProcess();

    // Map knob values [0..1] to useful reverb ranges.
    float time_cv = daisysp::fmap(hw.knobs[0]->Value(), 0.3f, 0.99f);
    float damp_cv = daisysp::fmap(hw.knobs[1]->Value(), 1000.f, 19000.f, daisysp::Mapping::LOG);

    verb.SetFeedback(time_cv);
    verb.SetLpFreq(damp_cv);

    // Process each sample in this audio block.
    for (size_t i = 0; i < size; i++)
    {
        // IN_L/IN_R and OUT_L/OUT_R are channel shortcuts from Daisy.
        verb.Process(IN_L[i], IN_R[i], &OUT_L[i], &OUT_R[i]);

        // Mix an oscillator voice into both output channels.
        float voice = vco.Process() * (hw.knobs[2]->Value() - 0.05);
        OUT_L[i] += voice;
        OUT_R[i] += voice;
    }
}

// Runs on the DAC/CV thread to fill CV output buffers.
static void CVOutCallback(uint16_t **out, size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        // Sum all CV inputs.
        float cvsum = 0.0f;
        for (auto* cvin : hw.cvins) { cvsum += cvin->Value(); }

        // Convert normalized float to DAC range (12-bit via >> 4).
        out[1][i] = static_cast<uint16_t>(cvsum * MAX_U16_FLOAT) >> 4;

        // Send LFO waveform to CV output 1.
        out[0][i] = static_cast<uint16_t>(lfo.Process()) >> 4;
    }
}

int main(void)
{
    // Bring up hardware and start real-time callbacks.
    hw.Init();
    hw.StartAudio(AudioCallback);
    hw.StartCV(CVOutCallback);

    // Audio-rate oscillator for audible tone generation.
    vco.Init(hw.som.AudioSampleRate());
    vco.SetFreq(440.0f);

    // CV-rate oscillator for modulation output.
    lfo.Init(hw.CvOutSampleRate());    
    lfo.SetFreq(220.0f);
    lfo.SetAmp(MAX_U16_FLOAT);

    // Reverb uses the same sample rate as the audio engine.
    verb.Init(hw.som.AudioSampleRate());

    // Optional debug logging.
    hw.som.StartLog(false);
    hw.som.PrintLine("Hello world");

    // Main control/UI loop (separate from audio callback).
    while(1) {
        // Show knob positions on the first LEDs.
        for (unsigned i = 0; i < hw.knobs.size(); i++) {
            hw.leds[i].Set(hw.knobs[i]->Value());
        }

#if __CCAM_TARGET_PLATFORM__ == PLATFORM_EARTH
        // Earth has extra LEDs; keep them lit by default.
        hw.leds[6].Set(1.0f);
        hw.leds[7].Set(1.0f);

        // Button press turns off its matching LED.
        for (unsigned i = 0; i < hw.leds.size(); i++) {
            if (hw.buttons[i].Pressed()) {
                hw.leds[i].Set(0.0f);
            }
        }
#endif

        // Small sleep so this loop does not busy-spin too hard.
        daisy::System::Delay(1);
    }
}