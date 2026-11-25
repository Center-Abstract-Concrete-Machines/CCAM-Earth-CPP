#pragma once
#ifndef ESTUARY_HARDWARE_INTERFACE_H
#define ESTUARY_HARDWARE_INTERFACE_H

#include "Settings.h"
#include "State.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * @brief EstuaryHardwareInterface (libDaisy / libCCAM port)
 * Option A (Direct 1-Page Eurorack) + Overdub Mode + CV/Gate Outputs + Cubic-Gamma LEDs.
 *
 * Knobs (0..7, zero menu diving):
 *   Knob 0: Grain Size (80 ms to 750 ms)       + CV 1
 *   Knob 1: Grain Density (1 to 32 voices)     + CV 2
 *   Knob 2: Grain Arp Reach ([-12, +24] st)    + CV 3
 *   Knob 3: Randomness (jitter, detune, octave leaps, loop window)
 *   Knob 4: Filter / Diffusion Macro           + CV 4
 *   Knob 5: Reverb Feedback (0.10 to 0.985)    + CV 4
 *   Knob 6: Verb Mix (Equal-Power)
 *   Knob 7: Dry / Wet Mix (Equal-Power +6dB wet)
 *
 * Switches:
 *   Switch 1 (switches[0]): Arp Scale (UP/1: Fifths, CENTER/0: Pentatonic/Add9, DOWN/2: Custom Just)
 *   Switch 2 (switches[1]): Grain Direction (UP/1: Forward, CENTER/0: 50% Random, DOWN/2: Reverse)
 *
 * Gates:
 *   Gate In 1: Freeze (active while HIGH)
 *   Gate In 2: Overdub Mode (active while HIGH, 95% continuous loop overdub)
 *
 * Outputs:
 *   CV Out 1:   Unused
 *   CV Out 2:   Pan LFO (0..5V)
 *   Gate Out 1: Freeze Active
 *   Gate Out 2: Overdub Active
 *   LEDs 0..7:  libDaisy software-PWM knob/modulation feedback;
 *               LED 6 locks ON when Freeze is active; LED 7 locks ON when Overdub is active.
 */
class EstuaryHardwareInterface {
public:
  EstuaryHardwareInterface() = default;
  ~EstuaryHardwareInterface() = default;

  void Init() {
    first_analog_read_ = true;
    for (int i = 0; i < 8; ++i) {
      sm_knobs_[i] = 0.0f;
    }
    for (int i = 0; i < 4; ++i) {
      sm_cvs_[i] = 0.0f;
    }
  }

  /**
   * @brief Blocking startup LED chase animation. Call in main() prior to StartAudio().
   */
  template <typename EstuaryT>
  void PlayStartupSequence(EstuaryT &estuary) {
    for (int i = 0; i < 8; ++i) {
      for (int j = 0; j < 8; ++j) {
        estuary.leds[j].Set(j == i ? 1.0f : 0.0f);
      }
      for (uint32_t ms = 0; ms < STARTUP_LED_STEP_MS; ++ms) {
        estuary.PostProcess();
        daisy::System::Delay(1);
      }
    }
    for (int i = 0; i < 8; ++i) {
      estuary.leds[i].Set(1.0f);
    }
    for (uint32_t ms = 0; ms < STARTUP_LED_STEP_MS * 2; ++ms) {
      estuary.PostProcess();
      daisy::System::Delay(1);
    }
    for (int i = 0; i < 8; ++i) {
      estuary.leds[i].Set(0.0f);
    }
    estuary.PostProcess();
  }

  /**
   * @brief Process 3-way switches and Gate inputs at 1000 Hz in main loop.
   * Gate In 1 = Freeze (priority hold); Gate In 2 = Overdub (95% continuous feedback).
   */
  template <typename EstuaryT>
  void ProcessDigitalControls(EstuaryT &estuary, GranulatorState &state) {
    // Switch 1: Arp Scale Quantization (libDaisy Switch3::Read(): 1=UP, 0=CENTER, 2=DOWN)
    const int arp_sw = estuary.switches[0].Read();
    state.arpMode = (arp_sw == 1) ? ArpMode::FIFTHS
                  : (arp_sw == 2) ? ArpMode::CUSTOM
                                  : ArpMode::CHROMATIC;

    // Switch 2: Grain Playback Direction
    const int dir_sw = estuary.switches[1].Read();
    state.grainDirection = (dir_sw == 1) ? GrainDirection::FORWARD
                         : (dir_sw == 2) ? GrainDirection::REVERSE
                                         : GrainDirection::RANDOM;

    // Signal flow is fixed to Granulator -> Reverb in Option A
    state.flowMode = SignalFlowMode::GRAN_TO_VERB;
    state.bypass = false;

    // Gate Inputs: Gate 1 = Freeze, Gate 2 = Overdub
    const bool g1 = estuary.GateIn1().State();
    const bool g2 = estuary.GateIn2().State();
    state.gate1Freeze = g1;
    state.gate2Overdub = g2;

    if (g1) {
      state.isFrozen = true;
      state.freezeMode = FreezeMode::MOMENTARY;
    } else if (g2) {
      state.isFrozen = true;
      state.freezeMode = FreezeMode::OVERDUB;
    } else {
      state.isFrozen = false;
      state.freezeMode = FreezeMode::MOMENTARY;
    }
  }

  /**
   * @brief Process 8 knobs and 4 bipolar CV inputs at 100 Hz in main loop.
   * Applies EMA anti-zipper smoothing and maps CV modulation depths.
   */
  template <typename EstuaryT>
  void ProcessAnalogControls(EstuaryT &estuary, GranulatorState &state) {
    float raw_k[8];
    for (int i = 0; i < 8; ++i) {
      raw_k[i] = Clamp01(estuary.Knob(static_cast<uint8_t>(i)));
    }

    float raw_cv[4];
    for (int i = 0; i < 4; ++i) {
      raw_cv[i] = ClampBipolar(estuary.cvins[i]->Value());
    }

    if (first_analog_read_) {
      first_analog_read_ = false;
      for (int i = 0; i < 8; ++i) {
        sm_knobs_[i] = raw_k[i];
      }
      for (int i = 0; i < 4; ++i) {
        sm_cvs_[i] = raw_cv[i];
      }
    } else {
      for (int i = 0; i < 8; ++i) {
        sm_knobs_[i] += IO_SMOOTHING_COEFFICIENT * (raw_k[i] - sm_knobs_[i]);
      }
      for (int i = 0; i < 4; ++i) {
        sm_cvs_[i] += IO_SMOOTHING_COEFFICIENT * (raw_cv[i] - sm_cvs_[i]);
      }
    }

    state.cvSize = sm_cvs_[0];
    state.cvDensity = sm_cvs_[1];
    state.cvArp = sm_cvs_[2];
    state.cvSpace = sm_cvs_[3];

    // Knob 0 + CV 1: Grain Size
    state.grainSize = Clamp01(sm_knobs_[0] + sm_cvs_[0] * CV_MOD_SIZE_DEPTH);

    // Knob 1 + CV 2: Grain Density
    state.grainDensity = Clamp01(sm_knobs_[1] + sm_cvs_[1] * CV_MOD_DENSITY_DEPTH);

    // Knob 2 + CV 3: Grain Arp Reach
    state.grainArp = Clamp01(sm_knobs_[2] + sm_cvs_[2] * CV_MOD_ARP_DEPTH);

    // Knob 3: Randomness
    state.randomness = sm_knobs_[3];

    // Knob 4 + CV 4: Filter Bank Range & Schroeder/Cloud Diffusion Macro
    const float filter_macro = Clamp01(sm_knobs_[4] + sm_cvs_[3] * CV_MOD_SPACE_FILTER_DEPTH);
    state.filterDiffusion = filter_macro;
    state.filterRange = std::pow(filter_macro, 0.70f);
    state.diffusionAmount = filter_macro;

    // Knob 5 + CV 4: Reverb Feedback (0.10 to 0.985, clamped <= 0.99)
    state.verbFeedbackKnob = sm_knobs_[5];
    state.masterFeedback = Clamp01(sm_knobs_[5] + sm_cvs_[3] * CV_MOD_SPACE_FEEDBACK_DEPTH);
    const float fb_range = state.feedbackMax - state.feedbackMin;
    state.verbFeedback = state.feedbackMin + state.masterFeedback * fb_range;
    if (state.verbFeedback > 0.99f) {
      state.verbFeedback = 0.99f;
    }

    // Knob 6: Verb Mix (Equal-Power)
    state.verbMix = sm_knobs_[6];

    // Knob 7: Dry / Wet Mix (Equal-Power +6 dB wet makeup)
    state.dryWetMix = sm_knobs_[7];
  }

  /**
   * @brief Updates all 8 LEDs via libDaisy's cubic-gamma 120 Hz PWM (PostProcess() at 1000 Hz).
   */
  template <typename EstuaryT>
  void UpdateLEDs(EstuaryT &estuary, const GranulatorState &state) {
    const float levels[8] = {
        state.grainSize,
        state.grainDensity,
        state.grainArp,
        state.randomness,
        state.filterDiffusion,
        state.masterFeedback,
        state.gate1Freeze ? 1.0f : state.verbMix,
        state.gate2Overdub ? 1.0f : state.dryWetMix
    };

    for (int i = 0; i < 8; ++i) {
      float val = Clamp01(levels[i]);
      if (val >= 0.995f) {
        val = 1.0f;
      } else if (val > 0.04f && val < 0.25f) {
        // Tactile floor so low non-zero knob positions remain visibly lit under cubic gamma
        val = 0.25f;
      }
      estuary.leds[i].Set(val);
    }
    estuary.PostProcess();
  }

  /**
   * @brief Drives Estuary CV Outputs (0..5V) and Gate Outputs at 100 Hz in main loop.
   */
  template <typename EstuaryT>
  void UpdateOutputs(EstuaryT &estuary, const GranulatorState &state, float pan_lfo_val) {
    estuary.GateOut1().Write(state.gate1Freeze);
    estuary.GateOut2().Write(state.gate2Overdub);
    estuary.WriteCvOut(daisy::patch_sm::CV_OUT_2, Clamp01(pan_lfo_val) * 5.0f);
  }

  void ResetSmoothing() {
    first_analog_read_ = true;
    for (int i = 0; i < 8; ++i) sm_knobs_[i] = 0.0f;
    for (int i = 0; i < 4; ++i) sm_cvs_[i] = 0.0f;
  }

  float GetSmoothedKnob(int idx) const {
    return (idx >= 0 && idx < 8) ? sm_knobs_[idx] : 0.0f;
  }

  float GetSmoothedCv(int idx) const {
    return (idx >= 0 && idx < 4) ? sm_cvs_[idx] : 0.0f;
  }

private:
  static inline float Clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  }

  static inline float ClampBipolar(float v) {
    if (v < -1.0f) return -1.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  }

  float sm_knobs_[8] = {0.0f};
  float sm_cvs_[4] = {0.0f};
  bool first_analog_read_ = true;
};

#endif // ESTUARY_HARDWARE_INTERFACE_H
