#ifndef __CCAM_ESTUARY_H__
#define __CCAM_ESTUARY_H__

/**
 * CCAM Estuary Hardware Abstraction
 * 
 * CCAM Estuary is based on the Daisy Patch SM platform and provides access to:
 * 
 * Hardware Features:
 * - 8 Knobs (analog inputs, first 4 are bipolar CV capable)
 * - 4 CV Inputs (additional analog inputs for external control)
 * - 2 Gate Inputs (digital trigger/gate detection)
 * - 8 LEDs (programmable indicators)
 * - 2 Three-way Switches (up/center/down positions)
 * - 2 Audio Inputs + 2 Audio Outputs (24-bit, up to 96kHz)
 * - 2 CV Outputs (12-bit DAC for control voltage generation)
 * 
 * Target Use Case:
 * - Performance-oriented modules requiring extensive manual control
 * - Live performance and real-time manipulation
 * - Complex synthesis and effects processing
 * - Eurorack integration with abundant CV I/O
 * 
 * Comparison to CCAM Earth:
 * - More knobs (8 vs 6)
 * - More CV inputs (4 vs 2)
 * - Fewer buttons (2 switches vs 8 buttons)
 * - Same audio I/O and CV output capabilities
 */

#include <array>

#include "daisy_patch_sm.h"

namespace ccam {

namespace hw {

/**
 * CCAM Estuary Hardware Abstraction Layer
 * 
 * Provides unified access to all Daisy Patch SM-based I/O:
 * - Audio processing (Init, StartAudio)
 * - Control voltage generation (StartCV) 
 * - Real-time control reading (ProcessAllControls)
 * - Gate input detection (gate_in_1, gate_in_2)
 * - LED and switch management (PostProcess)
 */
struct Estuary {

    void Init()
    {
        som.Init();

        static constexpr std::array<daisy::Pin, 8> ledPins = {
            daisy::patch_sm::DaisyPatchSM::D2,
            daisy::patch_sm::DaisyPatchSM::D3,
            daisy::patch_sm::DaisyPatchSM::D4,
            daisy::patch_sm::DaisyPatchSM::D5,
            daisy::patch_sm::DaisyPatchSM::D7,
            daisy::patch_sm::DaisyPatchSM::D6,
            daisy::patch_sm::DaisyPatchSM::D1,
            daisy::patch_sm::DaisyPatchSM::D10
        };

        for (uint8_t i = 0; i < leds.size(); i++) {
            leds[i].Init(ledPins[i], false);
            leds[i].Set(0.0f);
        }

        uint8_t adc_idx = 0;
        for (uint8_t i = 0; i < cvins.size(); i++) {
            cvins[i] = &som.controls[adc_idx++];
        }

        for (uint8_t i = 0; i < knobs.size(); i++) {
            knobs[i] = &som.controls[adc_idx];
            if (i < 4) {
                // Knobs 0-3: routed through PatchSM 5V CV inputs (inverting op-amp)
                // InitBipolarCv compensates for the op-amp and gives 0.0-1.0 for 0-5V
                knobs[i]->InitBipolarCv(som.adc.GetPtr(adc_idx++), som.AudioCallbackRate());
            } else {
                // Knobs 4-7: routed through 3.3V ADC inputs (direct)
                knobs[i]->Init(som.adc.GetPtr(adc_idx++), som.AudioCallbackRate());
            }
        }

        switches[0].Init(
            daisy::patch_sm::DaisyPatchSM::B8,
            daisy::patch_sm::DaisyPatchSM::B7
        );

        switches[1].Init(
            daisy::patch_sm::DaisyPatchSM::A9,
            daisy::patch_sm::DaisyPatchSM::A8
        );

        // Initialize gate inputs
        gate_inputs[0] = &som.gate_in_1;
        gate_inputs[1] = &som.gate_in_2;
    }

    void PostProcess()
    {
        for (uint8_t i = 0; i < leds.size(); i++) {
            leds[i].Update();
        }
    }

    void ProcessAllControls()
    {
        som.ProcessAllControls();
        // Note: GateIn objects are processed automatically by the som.ProcessAllControls()
        // No additional processing needed for gate inputs
    }

    // --- Audio ---
    float AudioSampleRate() { return som.AudioSampleRate(); }
    float AudioCallbackRate() { return som.AudioCallbackRate(); }
    void SetAudioSampleRate(size_t sr) { som.SetAudioSampleRate(sr); }
    void SetAudioBlockSize(size_t bs) { som.SetAudioBlockSize(bs); }
    void StartAudio(daisy::AudioHandle::AudioCallback cb) { som.StartAudio(cb); }

    // --- Knobs (all return 0.0 to 1.0) ---
    // Knobs 0-3: 5V CV inputs with inverting op-amp. InitBipolarCv gives
    //   ~0.0 at CCW (0V) to ~0.65 at CW (3.3V). Scale to fill 0..1.
    // Knobs 4-7: 3.3V ADC inputs, Init outputs 0..1 directly
    float Knob(uint8_t i) {
        if (i < 4) {
            float v = knobs[i]->Value() * 1.55f;
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;
            return v;
        }
        return knobs[i]->Value();
    }

    // --- CV out ---
    void WriteCvOut(int channel, float voltage) { som.WriteCvOut(channel, voltage); }
    void StartCV(daisy::DacHandle::DacCallback cb) { som.StartDac(cb); }

    size_t CvOutSampleRate() const {
        return som.dac.GetConfig().target_samplerate;
    }

    size_t CvOutCallbackRate() const {
        return som.dac.GetConfig().target_samplerate / 48;
    }

    // --- Gate I/O ---
    daisy::GateIn& GateIn1() { return som.gate_in_1; }
    daisy::GateIn& GateIn2() { return som.gate_in_2; }
    daisy::GPIO& GateOut1() { return som.gate_out_1; }
    daisy::GPIO& GateOut2() { return som.gate_out_2; }

    // --- Debug ---
    void StartLog(bool wait = false) { som.StartLog(wait); }
    template<typename... VA>
    void PrintLine(const char* fmt, VA... va) { som.PrintLine(fmt, va...); }

    // --- Hardware ---
    daisy::patch_sm::DaisyPatchSM som;                 // Core Daisy Patch SM hardware interface
    std::array<daisy::AnalogControl*, 8> knobs;         // 8 knobs (0-3: bipolar CV, 4-7: unipolar)
    std::array<daisy::AnalogControl*, 4> cvins;         // 4 CV inputs for external control
    std::array<daisy::GateIn*, 2> gate_inputs;          // 2 gate inputs for trigger/gate detection
    std::array<daisy::Led, 8> leds;                     // 8 programmable LEDs
    std::array<daisy::Switch3, 2> switches;             // 2 three-way switches (up/center/down)
};

} // namespace hw

} // namespace ccam

#endif // __CCAM_ESTUARY_H__