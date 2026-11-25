#ifndef STATE_H
#define STATE_H

#include "Settings.h"

// --- Enums ---

enum class SignalFlowMode {
  GRAN_TO_VERB,
  VERB_TO_GRAN,
  PARALLEL
};

enum class ArpMode {
  CHROMATIC, // Switch 1 Center
  FIFTHS,    // Switch 1 Up
  CUSTOM     // Switch 1 Down
};

enum class FreezeMode {
  LATCHING,
  MOMENTARY,
  OVERDUB
};

enum class GrainDirection {
  FORWARD, // Switch 2 Up: every grain plays forward
  RANDOM,  // Switch 2 Center: each grain has a 50% chance of playing reversed
  REVERSE  // Switch 2 Down: every grain plays reversed
};

// --- Parameter Structure ---
// Holds all volatile state derived from Estuary knobs, CV inputs, switches, and gates
struct GranulatorState {
  // Effective Normalized Parameters (0.0 to 1.0, post-CV modulation)
  float grainSize = 0.5f;       // Knob 0 + CV 0: Grain Size
  float grainDensity = 0.5f;    // Knob 1 + CV 1: Grain Density / Target Overlap
  float grainArp = 0.0f;        // Knob 2 + CV 2: Granular Arpeggiator Spread/Reach
  float randomness = 0.0f;      // Knob 3: Start Jitter, Detune, Pitch Jumps, Freeze Window
  float filterDiffusion = 1.0f; // Knob 4 + CV 3: Filter Bank Range & Schroeder/Cloud Diffusion
  float verbFeedbackKnob = 0.5f;// Knob 5: Raw Reverb Feedback knob
  float verbMix = 0.35f;        // Knob 6: Equal-Power Verb Mix
  float dryWetMix = 0.5f;       // Knob 7: Master Dry/Wet Blend

  // Smoothed Bipolar CV Inputs (-1.0 to +1.0)
  float cvSize = 0.0f;
  float cvDensity = 0.0f;
  float cvArp = 0.0f;
  float cvSpace = 0.0f;

  // Derived DSP Parameters
  float diffusionAmount = MAX_DIFFUSION_AMOUNT;
  float verbFeedback = 0.5f;
  float masterFeedback = 0.5f;
  float filterRange = 1.0f;

  // Feedback Range Configuration
  float feedbackMin = 0.10f;
  float feedbackMax = 0.985f;

  // Switch States
  SignalFlowMode flowMode = SignalFlowMode::GRAN_TO_VERB;
  ArpMode arpMode = ArpMode::CHROMATIC;
  FreezeMode freezeMode = FreezeMode::MOMENTARY;
  GrainDirection grainDirection = GrainDirection::FORWARD;

  // Gate & Toggle States
  bool bypass = false;
  bool isFrozen = false;
  bool gate1Freeze = false;
  bool gate2Overdub = false;
};

#endif // STATE_H
