# Landscape Granulator (CCAM Estuary / libDaisy Port)

Ported from `AGA_DaisyDuino/Arduino_Sketches/estuary_landscape_granulator` to native **libDaisy + DaisySP + libCCAM** in `CCAM-Earth-CPP`.

## Signal Chain
`Stereo In -> Smart L/Stereo Summer -> 2.0s AXI-SRAM GranularBuffer -> 32-Voice Tukey RealtimeGranulator -> 4-Band LFO RampSvf Bank -> 4x Schroeder + 6x Cloud Allpass + Stereo Tape-Warble Reverb Tank -> Equal-Power Dry/Wet -> Master Compressor -> Output Ceiling`

## Build & Flash
```bash
CCAM_PROJECT=9_landscape_granulator make ccam_main
CCAM_PROJECT=9_landscape_granulator make program-dfu
```

## Offline Host Tests
```bash
9_landscape_granulator/tests/run_tests
```

## Controls (Direct 1-Page Eurorack)
- **Knob 0 + CV 1:** Grain Size (80 ms – 750 ms per pass, `pow(x, 1.4)`)
- **Knob 1 + CV 2:** Grain Density (1 – 32 voices, equal-power auto-gain)
- **Knob 2 + CV 3:** Arp Reach ([-12, +24] semitones)
- **Knob 3:** Randomness (time scatter, micro-detune, octave leaps, freeze window 0.25–2.0 s)
- **Knob 4 + CV 4:** Filter / Diffusion Macro (4-band RampSvf sweep + allpass diffusion)
- **Knob 5 + CV 4:** Reverb Feedback (0.10 – 0.985)
- **Knob 6:** Verb Mix (equal-power)
- **Knob 7:** Dry / Wet Mix (equal-power, +6 dB wet makeup)
- **Switch 1:** Arp Scale (`UP` = Fifths, `CENTER` = Pentatonic/Add9, `DOWN` = Custom Just)
- **Switch 2:** Grain Direction (`UP` = Forward, `CENTER` = 50% Random, `DOWN` = Reverse)
- **Gate In 1 / Gate Out 1:** Freeze (LED 6 locks ON)
- **Gate In 2 / Gate Out 2:** Overdub 95% feedback loop (LED 7 locks ON)
- **CV Out 2:** Pan LFO (0–5 V)
