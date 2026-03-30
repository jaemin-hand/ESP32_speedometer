#pragma GCC push_options
#pragma GCC optimize("O3")

#include "app_main.h"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Arduino.h>
#include "driver/twai.h"
#include "lvgl.h"
#include "pins_config.h"

#include "app_types.h"
#include "app_config.h"
#include "../can/can_manager.h"
#include "../distance/distance_manager.h"
#include "../fusion/fusion_manager.h"
#include "../gnss/gnss_manager.h"
#include "../lcd/jd9165_lcd.h"
#include "../touch/gt911_touch.h"
#include "../ui/ui_manager.h"

namespace {

constexpr const char *kFirmwareTag = "FW CAN_DIAG_2026-03-30_21_GNSS_TEST_TOGGLE";

constexpr int CAN_RX_PIN = 2; // == receiver RX label
constexpr int CAN_TX_PIN = 48; // == transceiver TX label
constexpr int GPS_RX_PIN = 3;  // mosaic TX -> esp GPIO3
constexpr int GPS_TX_PIN = 47; // esp GPIO47 -> mosaic RX

constexpr uint32_t UI_UPDATE_INTERVAL_MS = 33;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;
constexpr uint32_t CAN_STATUS_INTERVAL_MS = 1000;
constexpr bool ENABLE_CAN_HEARTBEAT = false;
constexpr uint32_t CAN_HEARTBEAT_INTERVAL_MS = 1000;
constexpr float GNSS_TEST_SPEED_KMH = 40.0f;
constexpr int GNSS_TEST_MODE = 1;
constexpr int GNSS_TEST_SATS = 10;

HardwareSerial gpsSerial(1);

jd9165_lcd lcd(LCD_RST);
gt911_touch touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

lv_disp_draw_buf_t drawBuf;
lv_color_t *buf = nullptr;
lv_color_t *buf1 = nullptr;
lv_disp_drv_t dispDrv;

GnssManager gnss;
CanManager canManager;
DistanceManager distanceManager;
FusionManager fusionManager;
UiManager uiManager;
bool gnssTestOverrideEnabled = true;

GpsData buildEffectiveGpsData(const GpsData &rawGps) {
  GpsData effectiveGps = rawGps;

  if (gnssTestOverrideEnabled) {
    effectiveGps.pvtValid = true;
    effectiveGps.pvtMode = GNSS_TEST_MODE;
    effectiveGps.satellites = GNSS_TEST_SATS;
    effectiveGps.speedKmh = GNSS_TEST_SPEED_KMH;
    effectiveGps.speedKnots = GNSS_TEST_SPEED_KMH / 1.852f;
  }

  return effectiveGps;
}

bool onLcdColorTransDone(esp_lcd_panel_handle_t panel,
                         esp_lcd_dpi_panel_event_data_t *edata,
                         void *userCtx) {
  (void)panel;
  (void)edata;

  lv_disp_drv_t *disp = static_cast<lv_disp_drv_t *>(userCtx);
  if (disp != nullptr) {
    lv_disp_flush_ready(disp);
  }
  return false;
}

void myDispFlush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *colorP) {
  (void)disp;
  lcd.lcd_draw_bitmap(
      area->x1,
      area->y1,
      area->x2 + 1,
      area->y2 + 1,
      &colorP->full);
}

void myTouchpadRead(lv_indev_drv_t *indevDriver, lv_indev_data_t *data) {
  (void)indevDriver;

  bool touched = false;
  uint16_t touchX = 0;
  uint16_t touchY = 0;
  touched = touch.getTouch(&touchX, &touchY);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

void printGpsSummary() {
  const GpsData gps = buildEffectiveGpsData(gnss.getData());
  const FusionState &fusionState = fusionManager.getState();
  const CanDecodedSpeedState &canSpeedState = canManager.getDecodedSpeedState();

  Serial.println("========== GPS SUMMARY ==========");
  Serial.printf(
      "GNSS Test   : %s%s\n",
      gnssTestOverrideEnabled ? "OFF" : "ON",
      gnssTestOverrideEnabled ? " (override speed active)" : "");
  Serial.printf("Valid      : %s\n", gps.pvtValid ? "YES" : "NO");
  Serial.printf(
      "Mode       : %d (%s)\n",
      gps.pvtMode,
      GnssManager::pvtStatusToText(gps.pvtMode, gps.errorCode));
  Serial.printf("SourceMode : %s\n", FusionManager::modeToText(fusionState.mode));
  Serial.printf("AutoState  : %s\n", FusionManager::autoStateToText(fusionState.autoState));
  Serial.printf("Selected   : %s\n", FusionManager::sourceToText(fusionState.selectedSource));
  Serial.printf(
      "Stable     : GNSS=%s CAN=%s EXT=%s\n",
      fusionState.gnssStable ? "YES" : "NO",
      fusionState.canStable ? "YES" : "NO",
      fusionState.extStable ? "YES" : "NO");
  Serial.printf(
      "Corr       : active=%s learned=%s factor=%.4f samples=%u correctedCAN=%.2f km/h\n",
      fusionState.corrActive ? "YES" : "NO",
      fusionState.corrLearned ? "YES" : "NO",
      fusionState.corrFactor,
      static_cast<unsigned>(fusionState.corrSampleCount),
      fusionState.correctedCanSpeedKmh);
  Serial.printf("CAN Decode : %s\n", canSpeedState.valid ? "YES" : "NO");
  if (canSpeedState.valid) {
    Serial.printf(
        "CAN Speed  : %.2f km/h (%s, 0x%X)\n",
        canSpeedState.speedKmh,
        canSpeedState.decoderName,
        canSpeedState.identifier);
  }
  Serial.printf("Sats       : %d\n", gps.satellites);
  Serial.printf("Speed      : %.2f km/h\n", distanceManager.getSelectedSpeedKmh());
  Serial.print("Lat        : ");
  Serial.println(gps.latitude, 8);
  Serial.print("Lon        : ");
  Serial.println(gps.longitude, 8);
  Serial.printf("Alt(MSL)   : %.2f m\n", gps.altitudeM);
  Serial.printf("Dist       : %.2f m\n", distanceManager.getDistanceMeters());
  Serial.printf("Date       : %s\n", strlen(gps.dateStr) ? gps.dateStr : "---- -- --");
  Serial.printf("Time(UTC)  : %s\n", strlen(gps.timeStr) ? gps.timeStr : "--:--:--");
  Serial.println("=================================");
}

UiSnapshot buildUiSnapshot(const GpsData &gps) {
  const FusionState &fusionState = fusionManager.getState();

  UiSnapshot snapshot;
  snapshot.extValid = false;
  snapshot.extSpeedKmh = 0.0f;
  snapshot.gpsValid = gps.pvtValid;
  snapshot.gpsSpeedKmh = gps.speedKmh;
  snapshot.canValid = canManager.hasDecodedSpeed();
  snapshot.canSpeedKmh = canManager.getDecodedSpeedKmh();
  snapshot.distanceMeters = distanceManager.getDistanceMeters();
  snapshot.tripElapsedMs = distanceManager.getTripElapsedMs();
  snapshot.sourceMode = fusionState.mode;
  snapshot.selectedSource = fusionState.selectedSource;
  snapshot.corrActive = fusionState.corrActive;
  snprintf(snapshot.canMonitorText, sizeof(snapshot.canMonitorText), "%s", canManager.getMonitorText());
  snapshot.gps = gps;
  return snapshot;
}

void handleUiActions(uint8_t actions, uint32_t nowMs) {
  if ((actions & UI_ACTION_SEND_TEST_CAN) != 0U) {
    if (canManager.sendTestFrame()) {
      Serial.println("SEL pressed -> CAN test frame sent");
    } else {
      Serial.println("SEL pressed -> CAN test frame failed");
    }
  }

  if ((actions & UI_ACTION_SOURCE_CAL) != 0U) {
    fusionManager.cycleMode();
    Serial.printf(
        "Source/CAL pressed -> source mode = %s\n",
        FusionManager::modeToText(fusionManager.getState().mode));
  }

  if ((actions & UI_ACTION_RESET_DISTANCE) != 0U) {
    distanceManager.reset(nowMs);
    Serial.println("Trip reset");
  }
}

}  // namespace

void appSetup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32P4 speed display prototype");
  Serial.println(kFirmwareTag);

  lcd.begin();
  touch.begin();

  lv_init();

  const size_t bufferSize = sizeof(lv_color_t) * LCD_H_RES * LCD_V_RES;
  buf = static_cast<lv_color_t *>(heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM));
  buf1 = static_cast<lv_color_t *>(heap_caps_malloc(bufferSize, MALLOC_CAP_SPIRAM));
  assert(buf != nullptr);
  assert(buf1 != nullptr);

  lv_disp_draw_buf_init(&drawBuf, buf, buf1, LCD_H_RES * LCD_V_RES);

  lv_disp_drv_init(&dispDrv);
  dispDrv.hor_res = LCD_H_RES;
  dispDrv.ver_res = LCD_V_RES;
  dispDrv.flush_cb = myDispFlush;
  dispDrv.draw_buf = &drawBuf;
  dispDrv.full_refresh = false;
  lv_disp_drv_register(&dispDrv);

  if (!lcd.registerColorTransferDoneCallback(onLcdColorTransDone, &dispDrv)) {
    Serial.println("LCD flush callback registration failed");
  }

  static lv_indev_drv_t indevDrv;
  lv_indev_drv_init(&indevDrv);
  indevDrv.type = LV_INDEV_TYPE_POINTER;
  indevDrv.read_cb = myTouchpadRead;
  lv_indev_drv_register(&indevDrv);

  gnss.begin(gpsSerial, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GNSS serial ready for SBF");
  if (gnssTestOverrideEnabled) {
    Serial.printf(
        "GNSS test override active: %.1f km/h, mode=%d, sats=%d\n",
        GNSS_TEST_SPEED_KMH,
        GNSS_TEST_MODE,
        GNSS_TEST_SATS);
  }

  if (canManager.begin(
          static_cast<gpio_num_t>(CAN_TX_PIN),
          static_cast<gpio_num_t>(CAN_RX_PIN),
          AppConfig::kRequestedCanBackend,
          AppConfig::kActiveCanProfile)) {
    const CanBackendCapabilities caps = canManager.getBackendCapabilities();
    const CanBackendRequirements reqs = canManager.getBackendRequirements();
    Serial.println("CAN(TWAI) initialized");
    Serial.printf(
        "CAN backend requested: %s\n",
        (AppConfig::kRequestedCanBackend == CAN_BACKEND_CLASSIC) ? "CLASSIC_CAN" : "CAN_FD");
    Serial.printf("CAN backend in use: %s\n", canManager.getBackendName());
    Serial.printf("CAN profile in use: %s\n", canManager.getProfileName());
    Serial.printf("CAN profile note: %s\n", canManager.getProfileBringupNote());
    Serial.printf(
        "CAN backend caps: classic=%s fd=%s ready=%s\n",
        caps.supportsClassicCan ? "YES" : "NO",
        caps.supportsCanFd ? "YES" : "NO",
        caps.backendReady ? "YES" : "NO");
    Serial.printf(
        "CAN backend reqs: ext_ctrl=%s ext_xcvr=%s max_payload=%u driver=%s\n",
        reqs.requiresExternalController ? "YES" : "NO",
        reqs.requiresExternalTransceiver ? "YES" : "NO",
        static_cast<unsigned>(reqs.maxPayloadBytes),
        reqs.driverFamily);
    Serial.printf("CAN backend hw: %s\n", reqs.expectedHardware);
    Serial.printf("CAN next step : %s\n", reqs.nextBringupStep);
    Serial.printf("CAN backend note: %s\n", canManager.getBackendDiagnosticText());
  } else {
    Serial.println("CAN(TWAI) initialization failed");
    Serial.printf("CAN backend note: %s\n", canManager.getBackendDiagnosticText());
  }

  uiManager.begin();
}

void appLoop() {
  static unsigned long lastPrintMs = 0;
  static unsigned long lastCanStatusMs = 0;
  static unsigned long lastUiUpdateMs = 0;
  static unsigned long lastCanHeartbeatMs = 0;

  const uint32_t nowMs = millis();

  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch == 'g' || ch == 'G') {
      gnssTestOverrideEnabled = !gnssTestOverrideEnabled;
      Serial.printf(
          "GNSS test override -> %s (%.1f km/h, mode=%d, sats=%d)\n",
          gnssTestOverrideEnabled ? "ON" : "OFF",
          GNSS_TEST_SPEED_KMH,
          GNSS_TEST_MODE,
          GNSS_TEST_SATS);
    }
  }

  canManager.poll(nowMs);

  if (ENABLE_CAN_HEARTBEAT && (nowMs - lastCanHeartbeatMs) >= CAN_HEARTBEAT_INTERVAL_MS) {
    lastCanHeartbeatMs = nowMs;
    if (canManager.sendTestFrame()) {
      Serial.println("CAN TX heartbeat: 0x777 [4] 01 02 03 04");
    } else {
      Serial.println("CAN TX heartbeat failed");
      canManager.printStatus("CAN diagnostics");
    }
  }

  gnss.update();
  const GpsData gps = buildEffectiveGpsData(gnss.getData());

  FusionInputs fusionInputs;
  fusionInputs.nowMs = nowMs;
  fusionInputs.gnssValid = gps.pvtValid;
  fusionInputs.gnssMode = gps.pvtMode;
  fusionInputs.gnssSatellites = gps.satellites;
  fusionInputs.gnssSpeedKmh = gps.speedKmh;
  fusionInputs.canValid = canManager.hasDecodedSpeed();
  fusionInputs.canSpeedKmh = canManager.getDecodedSpeedKmh();
  fusionInputs.extValid = false;
  fusionInputs.extSpeedKmh = 0.0f;

  fusionManager.update(fusionInputs);
  distanceManager.update(nowMs, fusionManager.getState().selectedSpeedKmh);

  if ((nowMs - lastPrintMs) >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    printGpsSummary();
  }

  if ((nowMs - lastCanStatusMs) >= CAN_STATUS_INTERVAL_MS) {
    lastCanStatusMs = nowMs;
    canManager.printStatus("CAN status");
  }

  if ((nowMs - lastUiUpdateMs) >= UI_UPDATE_INTERVAL_MS) {
    lastUiUpdateMs = nowMs;
    uiManager.update(buildUiSnapshot(gps));
  }

  lv_timer_handler();
  handleUiActions(uiManager.consumeActions(), nowMs);
  delay(1);
}

#pragma GCC pop_options

