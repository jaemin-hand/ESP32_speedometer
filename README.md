![20260330_154026(1)](https://github.com/user-attachments/assets/c4e95141-0d43-448b-8baf-bc9a3cd74879)

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

- `ESP32_speedometer.ino`: application entry point and main loop
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

## Current feature status

- Speed sources
  - `GNSS`: Septentrio mosaic-X5 SBF input over UART
  - `CAN`: Classic CAN through ESP32-P4 TWAI
  - `EXT`: external pulse input on `GPIO46`
- Source selection
  - Manual modes: `AUTO`, `GNSS`, `CAN`, `EXT`
  - `AUTO` currently supports:
    - GNSS stable detection
    - CAN fallback
    - EXT fallback
    - GNSS/CAN correction-factor learning
- GNSS UI
  - `SATs` stale hold + gray rendering
  - `SPDQ` speed quality status
  - `LINK` GNSS stream freshness status
  - `Trip / Local` time display toggle
- Local time
  - Geographic timezone inference is implemented with bounding-box rules
  - Current implementation is heuristic, not a full timezone database
  - DST is not handled yet

## Current support scope

- Classic CAN
  - Raw CAN monitor: available
  - CAN speed decode: currently validated for `SantaFe Classic CAN`
  - Active speed candidates:
    - `0x450` byte0 replay speed
    - `0x386` wheel-average candidate
- CAN profile auto-detect
  - Runtime structure exists and tracks supported profiles
  - In practice, Classic CAN currently has only the Santa Fe profile implemented
  - Additional vehicles still require their own raw CAN logs and per-vehicle decoder rules
- CAN-FD
  - Project structure and placeholders exist
  - Actual CAN-FD receive path is not implemented yet
  - Planned direction: external CAN-FD controller such as `MCP2518FD`
- Pulse input (`EXT`)
  - Implemented and tested with switch/breadboard pulse injection
  - Current default config is defined in `src/app/app_config.h`

## Known limitations

- `SantaFe Classic CAN` is the only Classic CAN vehicle profile currently validated for speed decoding.
- New vehicle support requires raw CAN logging and decoder analysis before speed conversion can be added.
- mosaic-X5 SBF output is currently most reliable during GNSS/AUTO testing when the board is used with USB-C connected.
- `Local` time is location-inferred with coarse geo rules, not legal timezone-border precision.
- `EXT` currently uses `GPIO46`, which may later conflict with the planned MCP2518FD SPI pin plan and can be reassigned in `src/app/app_config.h`.

## Recommended test procedure

### GNSS

1. Connect mosaic-X5 over UART and USB-C for stable SBF testing.
2. Confirm `PVTGeodetic + ReceiverTime` are streaming on `COM1`.
3. Verify:
   - `SATs`
   - `SPDQ`
   - `LINK`
   - `Local` time

### Classic CAN

1. Replay or sniff Classic CAN at `500 kbps`.
2. Open the CAN monitor page and confirm raw frames appear.
3. For Santa Fe validation, send or observe:
   - `0x450`
   - `0x386`
4. Confirm:
   - `CAN Decode : YES`
   - `CAN Speed : ... km/h`

### AUTO

1. Verify `GNSS only -> USING GNSS`
2. Verify `CAN only -> USING CAN`
3. Verify `EXT only -> USING EXT`
4. Verify GNSS/CAN correction factor learns when both are stable above the learning threshold
5. Verify GNSS loss causes fallback to CAN or EXT as expected

### EXT pulse input

1. Inject pulses into `GPIO46`
2. Confirm `MODE EXT` shows EXT speed
3. Confirm `AUTO` can enter `EXT_FALLBACK`
4. Tune in `src/app/app_config.h` as needed:
   - `wheelCircumferenceMeters`
   - `pulsesPerWheelRevolution`
   - `sampleWindowMs`
   - `timeoutMs`
   - `minPulseIntervalUs`
   - `speedFilterAlpha`

