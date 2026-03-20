# CCAM Estuary "Hello World"

## About

This repository serves as a jumping off point for C++ projects built on the CCAM Estuary module (Daisy Patch SM based) and the CCAM Earth standalone synth. The setup has been tested and confirmed working on Windows with the DaisyToolchain.

**Hardware Configuration:**

- **CCAM Estuary** = 12v+/- Eurorack format synthesizer module
  - Daisy Patch SM based
  - 8 Knobs (first 4 bipolar CV capable)
  - 4 CV Inputs (external control)
  - 2 3-way switches
  - 2 CV Outputs
  - 8 LEDs + 2 Three-way Switches
  - 2 Audio I/O
  - **CCAM Earth** = 3.3v compact standalone desktop synth
  - 6 Knobs (analog inputs with CV capability)
  - 2 CV Inputs (external control)
  - 8 LEDs + 8 Buttons
  - 2 Audio I/O + 2 CV Outputs

## Setup

### Prerequisites

- **Windows** with [DaisyToolchain](https://github.com/electro-smith/DaisyWiki/wiki/1.-Setting-Up-Your-Development-Environment#Windows) installed
- **Git** for cloning repositories

### Clone and Build

```
git clone git@github.com:Center-Abstract-Concrete-Machines/CCAM-Earth-CPP.git --recurse-submodules

cd CCAM-Earth-CPP

```

## Build the Project

CCAM_PROJECT=0_kitchen_sink make
(replace '0_kitchen_sink' with whatever your project is called)

## Flash to Hardware

**Important:** This setup uses `APP_TYPE=BOOT_NONE` which flashes directly to internal flash memory.

### Step 1: Put Daisy Patch SM in DFU Mode

1. Hold the **BOOT** button on your Daisy Patch SM
2. Press and release the **RESET** button (while still holding BOOT)
3. Release the **BOOT** button
4. Connect USB cable to your computer

### Step 2: Verify Device Detection

```
dfu-util -l
```

You should see: `Found DFU: [0483:df11]` indicating the device is in DFU mode.

### Step 3: Flash Your Code

1

```
# Set target program
export CCAM_PROJECT=0_kitchen_sink

# Flash code directly to internal flash
make program-dfu
```

The device will automatically start running your code after flashing completes.

## Troubleshooting

### Device Not Detected

Run `dfu-util -l` to ensure the hardware is in DFU mode and the computer can see it. If that fails:

1. Try resetting the Daisy Patch SM and re-entering DFU mode
2. Check USB cable connection
3. Verify DaisyToolchain installation includes dfu-util

### Build Errors

- Ensure all submodules are cloned: `git submodule update --init --recursive`
- Clean and rebuild: `CCAM_PROJECT=0_kitchen_sink make clean && CCAM_PROJECT=0_kitchen_sink make`

### No Audio Output

1. Turn up **knob 2** (3rd knob from left) to hear the 440Hz VCO tone
2. Check audio connections to your mixer/speakers
3. Verify LEDs respond to knob movements (indicates code is running)

## Technical Notes

- **APP_TYPE**: `BOOT_NONE` (flashes directly to internal flash)
- **Platform**: `PLATFORM_ESTUARY` (configured for Daisy Patch SM hardware)
- **Memory Usage**: ~65% of internal flash (85KB/128KB)
- **Audio Processing**: Fixed buffer access (`in[0][i]`, `out[0][i]` format)
- **Serial Logging**: Disabled to prevent blocking issues

## Creating New Instruments

To create a new project:

1. **Copy existing project:**

   ```
   cp -r 0_kitchen_sink 0_my_new_instrument
   ```

2. **Update the Makefile:**

   ```
   TARGET = my_new_instrument
   # Keep other settings the same
   ```

3. **Edit main.cpp** with your custom audio processing and control mapping

4. **Build and flash:**
   ```
   CCAM_PROJECT=0_my_new_instrument make
   CCAM_PROJECT=0_my_new_instrument make program-dfu
   ```

### Key Code Areas to Modify:

- **AudioCallback()**: Real-time audio processing (DSP, effects, synthesis)
- **CVOutCallback()**: Control voltage generation
- **Main loop**: LED control, button handling, UI logic
- **DSP initialization**: Configure oscillators, filters, effects parameters
