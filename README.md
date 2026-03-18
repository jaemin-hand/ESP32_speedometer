# esp32p4_arduino_mipi-dsi_lvgl

ESP32-P4 Arduino/LVGL dashboard prototype for a 1024x600 MIPI-DSI display with GT911 touch, GNSS input, and CAN speed input.

## Repository scope

- This folder is self-contained and can be moved into its own Git repository.
- The parent folders from the original vendor/demo package are not required for source resolution or compilation.
- The project still depends on your local Arduino/ESP32 toolchain and libraries.

## Hardware summary

- MCU: ESP32-P4
- Display: JD9165 MIPI-DSI LCD, 1024x600
- Touch: GT911 over I2C
- GNSS: UART input parsed as SBF blocks
- Vehicle bus: TWAI/CAN

Pin assignments are defined in [pins_config.h](./pins_config.h).

## Software stack

- Arduino framework
- ESP32 Arduino core for `esp32:esp32:esp32p4`
- LVGL v8
- ESP-IDF display, touch, I2C, and TWAI drivers through the ESP32 Arduino core

## Project layout

- `esp32p4_arduino_mipi-dsi_lvgl.ino`: application entry point and main loop
- `src/gnss`: SBF parser and GNSS state handling
- `src/can`: TWAI setup and CAN speed decoding
- `src/fusion`: active speed source selection
- `src/distance`: trip distance and elapsed time integration
- `src/ui`: LVGL dashboard UI
- `src/lcd`: JD9165 LCD wrapper
- `src/touch`: GT911 touch wrapper
- `src/app`: shared application data types

## Runtime flow

```text
CAN poll -> GNSS update -> speed source selection -> distance integration -> UI snapshot -> LVGL render
```

Detailed flow:

1. Poll CAN frames
2. Read and parse GNSS bytes
3. Build `FusionInputs`
4. Select the active speed source
5. Integrate distance and trip time
6. Build `UiSnapshot`
7. Update LVGL widgets and flush to LCD

## Build requirements

- Arduino CLI or Arduino IDE
- ESP32 core installed for Arduino
- LVGL v8 installed in the Arduino libraries folder

Example compile command:

```bash
arduino-cli compile --clean --fqbn esp32:esp32:esp32p4 --board-options ChipVariant=prev3,JTAGAdapter=default,PSRAM=enabled,USBMode=default,CDCOnBoot=default,MSCOnBoot=default,DFUOnBoot=default,UploadMode=default,PartitionScheme=default,FlashMode=qio,FlashFreq=80,FlashSize=16M,UploadSpeed=921600,DebugLevel=none,EraseFlash=none .
```

## GitHub portability notes

- `lv_conf.h` is now local to this repository and no longer depends on an absolute path under `Documents/Arduino`.
- `.vscode` settings were cleaned up to reduce machine-specific values, but you may still need to adjust the serial port or regenerate IntelliSense data after board core updates.
- `debug.log` and VS Code cache files are ignored by `.gitignore`.

## Current implementation notes

- The `EXT` speed source is still a placeholder.
- The CAN speed decoder table still contains a template entry and should be updated with the real vehicle signal mapping.
