#ifndef __CCAM_EARTH_H__
#define __CCAM_EARTH_H__

/**
 * CCAM Earth Hardware Abstraction
 * 
 * CCAM Earth is based on the Daisy Seed platform and provides access to:
 * 
 * Hardware Features:
 * - 6 Knobs (analog inputs with CV capability)
 * - 2 CV Inputs (additional analog inputs for external control)
 * - 8 LEDs (programmable indicators)
 * - 8 Buttons (momentary switches with pullup)
 * - 2 Audio Inputs + 2 Audio Outputs (24-bit, up to 96kHz)
 * - 2 CV Outputs (12-bit DAC for control voltage generation)
 * 
 * Target Use Case:
 * - Compact modules requiring fewer physical controls
 * - Cost-optimized designs
 * - Projects focusing on audio processing over extensive manual control
 * 
 * Comparison to CCAM Estuary:
 * - Fewer knobs (6 vs 8)
 * - Fewer CV inputs (2 vs 4) 
 * - More buttons (8 vs 2 switches)
 * - Same audio I/O and CV output capabilities
 */

#include <array>

#include "daisy_seed.h"

static constexpr size_t cv_buf_len = 64;
uint16_t DMA_BUFFER_MEM_SECTION cvout1[cv_buf_len];
uint16_t DMA_BUFFER_MEM_SECTION cvout2[cv_buf_len];

namespace ccam {

namespace hw {

/**
 * CCAM Earth Hardware Abstraction Layer
 * 
 * Provides unified access to all Daisy Seed-based I/O:
 * - Audio processing (Init, StartAudio)
 * - Control voltage generation (StartCV)
 * - Real-time control reading (ProcessAllControls)
 * - LED and button management (PostProcess)
 */
struct Earth {

  /** Initializes the board according to the JSON board description
   *  \param boost boosts the clock speed from 400 to 480 MHz
   */
  void Init(bool boost=true) 
  {
    som.Configure();
    som.Init(boost);
 
    std::array<daisy::Pin, 8> ledPins = {
        daisy::seed::D7,
        daisy::seed::D8,
        daisy::seed::D9,
        daisy::seed::D10,
        daisy::seed::D11,
        daisy::seed::D12,
        daisy::seed::D13,
        daisy::seed::D14,
    };

    for (uint8_t i = 0; i < leds.size(); i++) {
        leds[i].Init(ledPins[i], false);
    }

    std::array<daisy::Pin, 8> buttonPins = {
        daisy::seed::D0,
        daisy::seed::D1,
        daisy::seed::D2,
        daisy::seed::D3,
        daisy::seed::D4,
        daisy::seed::D5,
        daisy::seed::D6,
        daisy::seed::D25,
    };

    for (uint8_t i = 0; i < buttons.size(); i++) {
        buttons[i].Init(buttonPins[i], som.AudioCallbackRate(), daisy::Switch::TYPE_MOMENTARY, daisy::Switch::POLARITY_INVERTED, daisy::GPIO::Pull::PULLUP);
    }
 
    std::array<daisy::AdcChannelConfig, 8> adcs;
    std::array<daisy::Pin, 8> adcPins = {
        daisy::seed::D17,
        daisy::seed::D18,
        daisy::seed::D19,
        daisy::seed::D20,
        daisy::seed::D21,
        daisy::seed::D24,
        daisy::seed::D15,
        daisy::seed::D16,
    };

    for (uint8_t i = 0; i < adcs.size(); i++) {
        adcs[i].InitSingle(adcPins[i]);
    }

    uint8_t adc_idx = 0;
    for (uint8_t i = 0; i < knobs.size(); i++) {
        knobs[i] = &controls[adc_idx++];
    }
    for (uint8_t i = 0; i < cvins.size(); i++) {
        cvins[i] = &controls[adc_idx++];
    }

    som.adc.Init(adcs.data(), adcs.size());

    for (uint8_t i = 0; i < adcs.size(); i++) {
        bool invert = (i < 6);
        controls[i].Init(som.adc.GetPtr(i), som.AudioCallbackRate(), invert, false);
    };

    cvout.bitdepth = daisy::DacHandle::BitDepth::BITS_12;
    cvout.buff_state = daisy::DacHandle::BufferState::ENABLED;
    cvout.mode = daisy::DacHandle::Mode::DMA;
    cvout.chn = daisy::DacHandle::Channel::BOTH;
    cvout.target_samplerate=48000;
    som.dac.Init(cvout);

    som.adc.Start();
  }

    void ProcessAllControls() 
    {
        for (uint8_t i = 0; i < knobs.size(); i++) {
            knobs[i]->Process();
        }
        for (uint8_t i = 0; i < cvins.size(); i++) {
            cvins[i]->Process();
        }
        for (uint8_t i = 0; i < buttons.size(); i++) {
            buttons[i].Debounce();
        }
    }

    void PostProcess()
    {
        for (uint8_t i = 0; i < leds.size(); i++) {
            leds[i].Update();
        }
    }

    void SetAudioSampleRate(size_t sample_rate) 
    {
        daisy::SaiHandle::Config::SampleRate enum_rate;
        if (sample_rate >= 96000)
            enum_rate = daisy::SaiHandle::Config::SampleRate::SAI_96KHZ;
        else if (sample_rate >= 48000)
            enum_rate = daisy::SaiHandle::Config::SampleRate::SAI_48KHZ;
        else if (sample_rate >= 32000)
            enum_rate = daisy::SaiHandle::Config::SampleRate::SAI_32KHZ;
        else if (sample_rate >= 16000)
            enum_rate = daisy::SaiHandle::Config::SampleRate::SAI_16KHZ;
        else
            enum_rate = daisy::SaiHandle::Config::SampleRate::SAI_8KHZ;
        som.SetAudioSampleRate(enum_rate);

        for (uint8_t i = 0; i < knobs.size(); i++) {
            knobs[i]->SetSampleRate(som.AudioCallbackRate());
        }
        for (uint8_t i = 0; i < cvins.size(); i++) {
            cvins[i]->SetSampleRate(som.AudioCallbackRate());
        }
        for (uint8_t i = 0; i < buttons.size(); i++) {
            buttons[i].SetUpdateRate(som.AudioCallbackRate());
        }
    }

    inline void SetAudioBlockSize(size_t block_size) 
    {
        som.SetAudioBlockSize(block_size);
    }

    inline void StartAudio(daisy::AudioHandle::AudioCallback cb)
    {
        som.StartAudio(cb);
    }

    inline void StartCV(daisy::DacHandle::DacCallback cb)
    {
        som.dac.Start(&cvout1[0], &cvout2[0], cv_buf_len, cb);
    }

    inline size_t CvOutSampleRate() {
        return som.dac.GetConfig().target_samplerate;
    }

    inline size_t CvOutCallbackRate() {
        return som.dac.GetConfig().target_samplerate / cv_buf_len;
    }

    daisy::DaisySeed som;                              // Core Daisy Seed hardware interface
    daisy::DacHandle::Config cvout;                    // CV output configuration
    std::array<daisy::AnalogControl, 8> controls;      // Raw analog control array (internal)
    std::array<daisy::AnalogControl*, 6> knobs;        // 6 knobs for user interface
    std::array<daisy::AnalogControl*, 2> cvins;        // 2 CV inputs for external control
    std::array<daisy::Led, 8> leds;                    // 8 programmable LEDs 
    std::array<daisy::Switch, 8> buttons;              // 8 momentary buttons
};

} // namespace hw

} // namspace ccam

#endif // __CCAM_EARTH_H__