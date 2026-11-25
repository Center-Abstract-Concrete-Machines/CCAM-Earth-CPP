#pragma once
#ifndef MOCK_LIBDAISY_H
#define MOCK_LIBDAISY_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace daisy {

namespace patch_sm {
enum {
  CV_OUT_BOTH = 0,
  CV_OUT_1,
  CV_OUT_2,
};
} // namespace patch_sm

class System {
public:
  static void Delay(uint32_t ms) { (void)ms; }
};

class AnalogControl {
public:
  void SetValue(float v) { val_ = v; }
  float Value() const { return val_; }
  float Process() { return val_; }

private:
  float val_ = 0.0f;
};

class GateIn {
public:
  void SetState(bool s) { state_ = s; }
  bool State() const { return state_; }

private:
  bool state_ = false;
};

class GPIO {
public:
  void Write(bool s) { state_ = s; }
  bool Read() const { return state_; }

private:
  bool state_ = false;
};

class Led {
public:
  void Init(uint8_t pin = 0, bool invert = false, float samplerate = 1000.0f) {
    (void)pin;
    (void)invert;
    (void)samplerate;
    val_ = 0.0f;
  }
  void Set(float val) { val_ = val; }
  void Update() {}
  float Get() const { return val_; }

private:
  float val_ = 0.0f;
};

class Switch3 {
public:
  enum {
    POS_CENTER = 0,
    POS_LEFT = 1,
    POS_UP = 1,
    POS_RIGHT = 2,
    POS_DOWN = 2,
  };

  void Set(int pos) { pos_ = pos; }
  int Read() const { return pos_; }

private:
  int pos_ = POS_CENTER;
};

} // namespace daisy

namespace ccam {
namespace hw {

struct Estuary {
  Estuary() {
    for (int i = 0; i < 8; ++i) {
      knobs[i] = &knob_objs_[i];
      leds[i].Init(static_cast<uint8_t>(i), false);
    }
    for (int i = 0; i < 4; ++i) {
      cvins[i] = &cv_objs_[i];
    }
  }

  void Init() {
    sample_rate_ = 48000.0f;
    block_size_ = 48;
  }
  void PostProcess() {}
  void ProcessAllControls() {}

  float AudioSampleRate() const { return sample_rate_; }
  float AudioCallbackRate() const { return sample_rate_ / static_cast<float>(block_size_); }
  void SetAudioBlockSize(size_t bs) { block_size_ = bs; }

  float Knob(uint8_t i) const { return (i < 8) ? knob_objs_[i].Value() : 0.0f; }
  void SetKnob(uint8_t i, float v) {
    if (i < 8) knob_objs_[i].SetValue(v);
  }
  void SetCv(uint8_t i, float v) {
    if (i < 4) cv_objs_[i].SetValue(v);
  }

  void WriteCvOut(int channel, float voltage) {
    if (channel == 1) cv_out_[0] = voltage;
    else if (channel == 2) cv_out_[1] = voltage;
  }
  float GetCvOut(int channel) const {
    return (channel == 1) ? cv_out_[0] : (channel == 2 ? cv_out_[1] : 0.0f);
  }

  daisy::GateIn &GateIn1() { return gates_[0]; }
  daisy::GateIn &GateIn2() { return gates_[1]; }
  daisy::GPIO &GateOut1() { return gate_outs_[0]; }
  daisy::GPIO &GateOut2() { return gate_outs_[1]; }
  bool GetGateOut1() const { return gate_outs_[0].Read(); }
  bool GetGateOut2() const { return gate_outs_[1].Read(); }

  std::array<daisy::AnalogControl *, 8> knobs;
  std::array<daisy::AnalogControl *, 4> cvins;
  std::array<daisy::Led, 8> leds;
  std::array<daisy::Switch3, 2> switches;

private:
  daisy::AnalogControl knob_objs_[8];
  daisy::AnalogControl cv_objs_[4];
  daisy::GateIn gates_[2];
  daisy::GPIO gate_outs_[2];
  float cv_out_[2] = {0.0f, 0.0f};
  float sample_rate_ = 48000.0f;
  size_t block_size_ = 48;
};

} // namespace hw
} // namespace ccam

#endif // MOCK_LIBDAISY_H
