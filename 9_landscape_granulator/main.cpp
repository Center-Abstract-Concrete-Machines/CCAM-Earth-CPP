#define __CCAM_TARGET_PLATFORM__ PLATFORM_ESTUARY
#include <ccam/hw/estuary.h>
#include "daisysp.h"

#include "Settings.h"
#include "State.h"
#include "AudioEngine.h"
#include "EstuaryHardwareInterface.h"

using namespace daisy;
using namespace daisysp;

// --- Estuary Hardware Abstraction & Global DSP Engine ---
static ccam::hw::Estuary estuary;
static GranulatorState state;
static EstuaryHardwareInterface hardware;
static AudioEngine engine;

// Callback load, peak cycles since last diagnostics read
static volatile uint32_t callback_peak_cycles = 0;

// --- Pure Audio Interrupt Callback ---
// Zero GPIO, zero ADC, zero debouncing inside interrupt.
// Chunked to BLOCK_SIZE = 48 inside ProcessBlock() for safe STM32 interrupt stack headroom.
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("O3")))
#endif
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size) {
  const uint32_t start_cycles = DWT->CYCCNT;

  engine.ProcessBlock(in, out, size, state);

  const uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
  if (elapsed_cycles > callback_peak_cycles) {
    callback_peak_cycles = elapsed_cycles;
  }
}

int main(void) {
  // 1. Initialize CCAM Estuary (Patch SM @ 480 MHz Boost, 48 kHz audio, MPU, SDRAM, ADC, DAC)
  estuary.Init();
  const float samplerate = estuary.AudioSampleRate();

  // 2. Enable ARM Cortex-M7 FPU Flush-to-Zero (FZ) and Default NaN (DN). FPSCR covers main();
  // interrupt handlers (the audio callback) start from FPDSCR instead, so set both.
  __set_FPSCR(__get_FPSCR() | (1U << 24) | (1U << 25));
  FPU->FPDSCR |= FPU_FPDSCR_FZ_Msk | FPU_FPDSCR_DN_Msk;

  if (ENABLE_DIAGNOSTICS) {
    estuary.StartLog(false);
  }

  // 3. Initialize Audio Engine & Hardware Interface prior to starting audio interrupt
  engine.Init(samplerate);
  hardware.Init();

  // 4. Startup LED chase sequence (blocking before audio DMA begins)
  hardware.PlayStartupSequence(estuary);

  // 5. Prime initial knob/switch/gate states before audio starts
  estuary.ProcessAllControls();
  hardware.ProcessDigitalControls(estuary, state);
  hardware.ProcessAnalogControls(estuary, state);

  // Enable DWT cycle counter for callback load measurement
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 6. Start Audio Stream with safe BLOCK_SIZE = 48
  estuary.SetAudioBlockSize(BLOCK_SIZE);
  estuary.StartAudio(AudioCallback);

  const uint32_t digital_tick_interval = System::GetTickFreq() / DIGITAL_CONTROLS_INTERVAL_HZ;
  uint32_t last_digital_tick = System::GetTick();
  uint32_t last_controls_ms = System::GetNow();
  uint32_t last_print_ms = last_controls_ms;

  while (1) {
    const uint32_t now_tick = System::GetTick();
    const uint32_t now_ms = System::GetNow();

    // Tier 1: Hardware Control Debouncing, Switch/Gate Decoding & 1000 Hz LED PWM
    if (now_tick - last_digital_tick >= digital_tick_interval) {
      last_digital_tick = now_tick;
      estuary.ProcessAllControls();
      hardware.ProcessDigitalControls(estuary, state);
      hardware.UpdateLEDs(estuary, state);
    }

    // Tier 2: Analog Knobs, Bipolar CV Inputs & CV/Gate Outputs (100 Hz)
    if (now_ms - last_controls_ms >= CONTROLS_UPDATE_INTERVAL_MS) {
      last_controls_ms = now_ms;
      hardware.ProcessAnalogControls(estuary, state);
      hardware.UpdateOutputs(estuary, state, engine.GetPanLfo());
      engine.UpdateParameters(state);
    }

    // Optional non-blocking diagnostics (compile-time disabled by default for flash headroom)
    if (ENABLE_DIAGNOSTICS && (now_ms - last_print_ms >= SERIAL_DIAGNOSTICS_MS)) {
      last_print_ms = now_ms;
      const float block_cycles = static_cast<float>(SystemCoreClock) * BLOCK_SIZE / samplerate;
      const float peak_pct = 100.0f * static_cast<float>(callback_peak_cycles) / block_cycles;
      callback_peak_cycles = 0;

      const char *arp_str = (state.arpMode == ArpMode::FIFTHS) ? "FIFTHS"
                          : (state.arpMode == ArpMode::CUSTOM) ? "CUSTOM"
                                                               : "CHROM";
      const char *dir_str = (state.grainDirection == GrainDirection::FORWARD) ? "FWD"
                          : (state.grainDirection == GrainDirection::RANDOM)  ? "RND"
                                                                              : "REV";

      estuary.PrintLine("--- Estuary Landscape Granulator ---");
      estuary.PrintLine("Peak Load: %d.%d%% | Arp: %s | Dir: %s | Frz: %d | Ovd: %d",
                        static_cast<int>(peak_pct),
                        static_cast<int>(peak_pct * 10.0f) % 10,
                        arp_str,
                        dir_str,
                        state.gate1Freeze ? 1 : 0,
                        state.gate2Overdub ? 1 : 0);
    }
  }
}
