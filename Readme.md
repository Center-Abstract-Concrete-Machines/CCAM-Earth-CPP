# CCAM Earth "Hello World"

## About
This repository is meant to serve primarily as a jumping off point for C++ projects built on the CCAM earth module. NOTE: I am running on Fedora linux, the steps will likely be fairly different on Mac/Windows.

## Patch
* Buttons
    * When pressed, the corresponding LED will turn off
* LEDs
    * 1 through 6: map directly to corresponding knob
    * 7 through 8: map directly to corresponding cvin
* CvOuts
    * 1: outputs 0-3V scaled linearly with knob1 value
    * 2: outputs lfo and fixed 220Hz
* CvIns
    * 1 through 2: map directly to LED 7 / 8
* Audio out
    * 1 through 2: reverb of audio in + vco (volume knob 6)
* Knobs
    * 1: linear output (cvout)
    * 2: lfo volume (cvout)
    * 4: reverb time (audio 1+2)
    * 5: reverb damping (audio 1+2)
    * 6: vco volume (audio 1+2)

## Setup
```
git clone git@github.com:Center-Abstract-Concrete-Machines/CCAM-Earth-CPP.git --recurse-submodules

cd CCAM-Earth-CPP

yum install dfu-util arm-none-eabi-gcc-cs arm-none-eabi-newlib

make
```

## Flash
I set APP_TYPE=BOOT_SRAM in Makefile, which allows us to use a number of bells and whistles (like print statements for example). This means we have to first load the bootloader program onto the board, and then flash our image on top.
```
# set target program
export CCAM_PROJECT=0_kitchen_sink

# upload bootloader
make program-boot

# build the code
make

# flash code to board
make program-dfu
```

## Debug
After you plug in the board, check `sudo dmesg | tail` to determine the serail port number. Then you can use screen to monitor the device.
```
sudo screen /dev/ttyACM0 115200
```
Note that the program will not start running until you connect, so if you do not plan on using the serial monitor it
it probably a good idea to remove that line entirely.

