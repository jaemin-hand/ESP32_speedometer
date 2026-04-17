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
#include "../ext/pulse_input_manager.h"
#include "../fusion/fusion_manager.h"
#include "../gnss/gnss_manager.h"
#include "../lcd/jd9165_lcd.h"
#include "../touch/gt911_touch.h"
#include "../ui/ui_manager.h"

namespace {

constexpr const char *kFirmwareTag = "FW CAN_DIAG_2026-04-06_37_MEASEPOCH_DIAG";

constexpr int GPS_RX_PIN = 3;  // mosaic TX -> esp GPIO3
constexpr int GPS_TX_PIN = 47; // esp GPIO47 -> mosaic RX pin
constexpr uint32_t GPS_BAUD_RATE = 460800;
constexpr size_t GPS_UART_RX_BUFFER_SIZE = 8192;

constexpr uint32_t UI_UPDATE_INTERVAL_MS = 33;
constexpr uint32_t DEBUG_PRINT_INTERVAL_MS = 1000;
constexpr uint32_t CAN_STATUS_INTERVAL_MS = 1000;
constexpr bool ENABLE_CAN_HEARTBEAT = false;
constexpr uint32_t CAN_HEARTBEAT_INTERVAL_MS = 1000;
constexpr float GNSS_TEST_SPEED_KMH = 125.0f;
constexpr float EXT_TEST_SPEED_KMH = 39.0f;
constexpr float GNSS_TEST_CN0_AVG_DBHZ = 42.0f;
constexpr float GNSS_TEST_CN0_MAX_DBHZ = 47.0f;
constexpr uint8_t GNSS_TEST_CN0_SIGNAL_COUNT = 12;
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
PulseInputManager pulseInputManager;
FusionManager fusionManager;
UiManager uiManager;
bool gnssTestOverrideEnabled = false;
bool extTestOverrideEnabled = false;
CanProfileId activeCanProfile = AppConfig::kActiveCanProfile;

GpsData buildEffectiveGpsData(const GpsData &rawGps) {
  GpsData effectiveGps = rawGps;

  if (gnssTestOverrideEnabled) {
    effectiveGps.pvtValid = true;
    effectiveGps.pvtMode = GNSS_TEST_MODE;
    effectiveGps.satellites = GNSS_TEST_SATS;
    effectiveGps.cn0Valid = true;
    effectiveGps.cn0SignalCount = GNSS_TEST_CN0_SIGNAL_COUNT;
    effectiveGps.cn0AvgDbHz = GNSS_TEST_CN0_AVG_DBHZ;
    effectiveGps.cn0MaxDbHz = GNSS_TEST_CN0_MAX_DBHZ;
    effectiveGps.sbfStreamActive = true;
    effectiveGps.receiverTimeValid = true;
    effectiveGps.sbfAgeMs = 0;
    effectiveGps.pvtAgeMs = 0;
    effectiveGps.cn0AgeMs = 0;
    effectiveGps.receiverTimeAgeMs = 0;
    effectiveGps.speedKmh = GNSS_TEST_SPEED_KMH;
    effectiveGps.speedKnots = GNSS_TEST_SPEED_KMH / 1.852f;
    effectiveGps.cn0_Max_Age_Ms = 0;
  }

  return effectiveGps;
}

PulseInputState buildEffectivePulseState(const PulseInputState &rawPulseState, uint32_t nowMs) {
  PulseInputState effectivePulseState = rawPulseState;

  if (extTestOverrideEnabled) {
    effectivePulseState.configured = true;
    effectivePulseState.valid = true;
    effectivePulseState.stale = false;
    effectivePulseState.speedKmh = EXT_TEST_SPEED_KMH;
    effectivePulseState.filteredSpeedKmh = EXT_TEST_SPEED_KMH;
    effectivePulseState.lastPulseMs = nowMs;
  }

  return effectivePulseState;
}

uint32_t applyLocalUtcOffset(uint32_t secondsOfDay, int32_t offsetMinutes) {
  int32_t shiftedSeconds =
      static_cast<int32_t>(secondsOfDay) + (offsetMinutes * 60);

  shiftedSeconds %= 86400;
  if (shiftedSeconds < 0) {
    shiftedSeconds += 86400;
  }

  return static_cast<uint32_t>(shiftedSeconds);
}

bool tryFormatLocalTimeFromUtc(
    const char *utcText,
    int32_t offsetMinutes,
    char *outBuf,
    size_t outBufSize) {
  if ((utcText == nullptr) || (strlen(utcText) < 8U)) {
    return false;
  }

  unsigned hour = 0;
  unsigned minute = 0;
  unsigned second = 0;
  if (sscanf(utcText, "%2u:%2u:%2u", &hour, &minute, &second) != 3) {
    return false;
  }

  const uint32_t localSeconds =
      applyLocalUtcOffset((hour * 3600U) + (minute * 60U) + second, offsetMinutes);
  snprintf(
      outBuf,
      outBufSize,
      "%02lu:%02lu:%02lu",
      static_cast<unsigned long>(localSeconds / 3600U),
      static_cast<unsigned long>((localSeconds / 60U) % 60U),
      static_cast<unsigned long>(localSeconds % 60U));
  return true;
}

GnssSpeedQuality classifyGnssSpeedQuality(const GpsData &gps) {
  if (!gps.pvtValid) {
    return GNSS_SPEED_QUALITY_LOST;
  }

  if (gps.cn0Valid && gps.cn0AgeMs <= 1500U) {
    if (gps.cn0AvgDbHz >= 40.0f && gps.cn0SignalCount >= 8U) {
      return GNSS_SPEED_QUALITY_HIGH;
    }
    if (gps.cn0AvgDbHz >= 34.0f && gps.cn0SignalCount >= 4U) {
      return GNSS_SPEED_QUALITY_MID;
    }
    return GNSS_SPEED_QUALITY_LOW;
  }

  switch (gps.pvtMode) {
    case 4:
    case 7:
      return (gps.satellites >= 8) ? GNSS_SPEED_QUALITY_HIGH : GNSS_SPEED_QUALITY_MID;
    case 5:
    case 8:
    case 10:
    case 2:
    case 6:
      return GNSS_SPEED_QUALITY_MID;
    case 1:
    case 3:
    default:
      return (gps.satellites >= 8) ? GNSS_SPEED_QUALITY_MID : GNSS_SPEED_QUALITY_LOW;
  }
}

GnssLinkQuality classifyGnssLinkQuality(const GpsData &gps) {
  if (!gps.sbfStreamActive) {
    return GNSS_LINK_QUALITY_LOST;
  }

  if (!gps.receiverTimeValid || !gps.pvtValid || gps.sbfAgeMs > 250U || gps.pvtAgeMs > 400U) {
    return GNSS_LINK_QUALITY_HOLD;
  }

  return GNSS_LINK_QUALITY_LIVE;
}

const char *gnssSpeedQualityToText(GnssSpeedQuality quality) {
  switch (quality) {
    case GNSS_SPEED_QUALITY_HIGH: return "HIGH";
    case GNSS_SPEED_QUALITY_MID: return "MID";
    case GNSS_SPEED_QUALITY_LOW: return "LOW";
    case GNSS_SPEED_QUALITY_LOST:
    default: return "LOST";
  }
}

const char *gnssLinkQualityToText(GnssLinkQuality quality) {
  switch (quality) {
    case GNSS_LINK_QUALITY_LIVE: return "LIVE";
    case GNSS_LINK_QUALITY_HOLD: return "HOLD";
    case GNSS_LINK_QUALITY_LOST:
    default: return "LOST";
  }
}

struct LocalTimeInfo {
  int32_t offsetMinutes = AppConfig::kLocalUtcOffsetMinutes;
  bool inferred = false;
  const char *label = "KST";
};

struct TimezoneGeoRule {
  double minLat;
  double maxLat;
  double minLon;
  double maxLon;
  int32_t offsetMinutes;
  const char *label;
};

constexpr TimezoneGeoRule kTimezoneGeoRules[] = {
    {32.0, 39.8, 124.0, 132.5, 9 * 60, "KST"},
    {24.0, 46.5, 122.0, 154.0, 9 * 60, "JST"},
    {20.0, 26.5, 119.0, 123.5, 8 * 60, "CST"},
    {18.0, 54.5, 73.0, 123.0, 8 * 60, "CST"},
    {4.0, 21.5, 116.0, 127.5, 8 * 60, "PHT"},
    {5.0, 21.0, 97.0, 106.5, 7 * 60, "ICT"},
    {8.0, 24.5, 102.0, 110.5, 7 * 60, "ICT"},
    {-11.0, 6.5, 95.0, 141.5, 7 * 60, "WIB"},
    {-11.0, 6.5, 113.0, 125.5, 8 * 60, "WITA"},
    {-11.0, 6.5, 125.0, 141.5, 9 * 60, "WIT"},
    {6.0, 37.5, 68.0, 98.5, 5 * 60 + 30, "IST"},
};

LocalTimeInfo inferLocalTimeInfo(const GpsData &gps) {
  if (gps.pvtValid && isfinite(gps.latitude) && isfinite(gps.longitude)) {
    for (const TimezoneGeoRule &rule : kTimezoneGeoRules) {
      if (gps.latitude >= rule.minLat && gps.latitude <= rule.maxLat &&
          gps.longitude >= rule.minLon && gps.longitude <= rule.maxLon) {
        return {
            .offsetMinutes = rule.offsetMinutes,
            .inferred = true,
            .label = rule.label,
        };
      }
    }
  }

  return {
      .offsetMinutes = AppConfig::kLocalUtcOffsetMinutes,
      .inferred = false,
      .label = "KST",
  };
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
  const CanProfileId canProfileId = canManager.getProfileId();
  const CanDecodedSpeedState &canSpeedState = canManager.getDecodedSpeedState();
  CanDecodedSpeedState canReplayState = {};
  CanDecodedSpeedState canWheelState = {};
  const bool isSantaFeProfile = (canProfileId == CAN_PROFILE_SANTAFE_CLASSIC);
  const bool hasCanReplayState =
      isSantaFeProfile &&
      canManager.getDecoderDiagnostic("santafe_replay_speed", &canReplayState);
  const bool hasCanWheelState =
      isSantaFeProfile &&
      canManager.getDecoderDiagnostic("santafe_wheel_avg_0x386", &canWheelState);
  const PulseInputState pulseState =
      buildEffectivePulseState(pulseInputManager.getState(), millis());
  const GnssSpeedQuality gnssSpeedQuality = classifyGnssSpeedQuality(gps);
  const GnssLinkQuality gnssLinkQuality = classifyGnssLinkQuality(gps);
  const LocalTimeInfo localTimeInfo = inferLocalTimeInfo(gps);
  char localTimeBuf[16] = {0};
  const bool hasLocalTime = tryFormatLocalTimeFromUtc(
      gps.timeStr,
      localTimeInfo.offsetMinutes,
      localTimeBuf,
      sizeof(localTimeBuf));

  Serial.println("========== GPS SUMMARY ==========");
  Serial.printf(
      "GNSS Test   : %s%s\n",
      gnssTestOverrideEnabled ? "ON" : "OFF",
      gnssTestOverrideEnabled ? " (override speed active)" : "");
  Serial.printf(
      "EXT Test    : %s%s\n",
      extTestOverrideEnabled ? "ON" : "OFF",
      extTestOverrideEnabled ? " (override speed active)" : "");
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
      "Corr       : active=%s learned=%s factor=%.4f samples=%u\n",
      fusionState.corrActive ? "YES" : "NO",
      fusionState.corrLearned ? "YES" : "NO",
      fusionState.corrFactor,
      static_cast<unsigned>(fusionState.corrSampleCount));
  Serial.printf(
      "Corr(CAN)  : learned=%s factor=%.4f samples=%u corrected=%.2f km/h\n",
      fusionState.canCorrLearned ? "YES" : "NO",
      fusionState.canCorrFactor,
      static_cast<unsigned>(fusionState.canCorrSampleCount),
      fusionState.correctedCanSpeedKmh);
  Serial.printf(
      "Corr(EXT)  : learned=%s factor=%.4f samples=%u corrected=%.2f km/h\n",
      fusionState.extCorrLearned ? "YES" : "NO",
      fusionState.extCorrFactor,
      static_cast<unsigned>(fusionState.extCorrSampleCount),
      fusionState.correctedExtSpeedKmh);
  Serial.printf("CAN Decode : %s\n", canSpeedState.valid ? "YES" : "NO");
  Serial.printf("CAN Profile : %s\n", canManager.getProfileName());
  if (canSpeedState.valid) {
    Serial.printf(
        "CAN Speed  : %.2f km/h (%s, 0x%X)\n",
        canSpeedState.speedKmh,
        canSpeedState.decoderName,
        canSpeedState.identifier);
  }
  if (hasCanReplayState || hasCanWheelState) {
    Serial.printf(
        "CAN Compare: 0x450=%s%.2f  0x386=%s%.2f",
        hasCanReplayState ? "" : "(n/a) ",
        hasCanReplayState ? canReplayState.speedKmh : 0.0f,
        hasCanWheelState ? "" : "(n/a) ",
        hasCanWheelState ? canWheelState.speedKmh : 0.0f);
    if (hasCanReplayState && hasCanWheelState) {
      Serial.printf(
          "  delta=%.2f",
          canWheelState.speedKmh - canReplayState.speedKmh);
    }
    Serial.println();
  }
  Serial.printf(
      "EXT Pulse  : configured=%s valid=%s stale=%s count=%lu speed=%.2f km/h filtered=%.2f km/h\n",
      pulseState.configured ? "YES" : "NO",
      pulseState.valid ? "YES" : "NO",
      pulseState.stale ? "YES" : "NO",
      static_cast<unsigned long>(pulseState.totalPulseCount),
      pulseState.speedKmh,
      pulseState.filteredSpeedKmh);
  Serial.printf(
      "GNSS Qual  : speed=%s link=%s sbfAge=%lu ms pvtAge=%lu ms timeAge=%lu ms\n",
      gnssSpeedQualityToText(gnssSpeedQuality),
      gnssLinkQualityToText(gnssLinkQuality),
      static_cast<unsigned long>(gps.sbfAgeMs),
      static_cast<unsigned long>(gps.pvtAgeMs),
      static_cast<unsigned long>(gps.receiverTimeAgeMs));
  Serial.printf(
      "GNSS C/N0  : valid=%s avg=%.1f dB-Hz max=%.1f dB-Hz n=%u age=%lu ms  Max_cn0AgeMs = %lu ms\n",
      gps.cn0Valid ? "YES" : "NO",
      gps.cn0AvgDbHz,
      gps.cn0MaxDbHz,
      static_cast<unsigned>(gps.cn0SignalCount),
      static_cast<unsigned long>(gps.cn0AgeMs),
      static_cast<unsigned long>(gps.cn0_Max_Age_Ms));
  Serial.printf(
      "Time Zone  : %s (%s, UTC%+ld:%02lu)\n",
      localTimeInfo.label,
      localTimeInfo.inferred ? "geo" : "fallback",
      static_cast<long>(localTimeInfo.offsetMinutes / 60),
      static_cast<unsigned long>(abs(localTimeInfo.offsetMinutes % 60)));
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
  Serial.printf("Time(Local): %s\n", hasLocalTime ? localTimeBuf : "--:--:--");
  Serial.println("=================================");
}

UiSnapshot buildUiSnapshot(const GpsData &gps) {
  const FusionState &fusionState = fusionManager.getState();
  const PulseInputState pulseState =
      buildEffectivePulseState(pulseInputManager.getState(), millis());

  UiSnapshot snapshot;
  snapshot.extValid = pulseState.valid;
  snapshot.extSpeedKmh = pulseState.speedKmh;
  snapshot.gpsValid = gps.pvtValid;
  snapshot.gpsSpeedKmh = gps.speedKmh;
  snapshot.canValid = canManager.hasDecodedSpeed();
  snapshot.canSpeedKmh = canManager.getDecodedSpeedKmh();
  snapshot.distanceMeters = distanceManager.getDistanceMeters();
  snapshot.tripElapsedMs = distanceManager.getTripElapsedMs();
  snapshot.sourceMode = fusionState.mode;
  snapshot.selectedSource = fusionState.selectedSource;
  snapshot.corrActive = fusionState.corrActive;
  snapshot.gnssSpeedQuality = classifyGnssSpeedQuality(gps);
  snapshot.gnssLinkQuality = classifyGnssLinkQuality(gps);
  const LocalTimeInfo localTimeInfo = inferLocalTimeInfo(gps);
  snapshot.localUtcOffsetMinutes = localTimeInfo.offsetMinutes;
  snapshot.localUtcOffsetInferred = localTimeInfo.inferred;
  snprintf(snapshot.canMonitorText, sizeof(snapshot.canMonitorText), "%s", canManager.getMonitorText());
  snapshot.gps = gps;
  return snapshot;
}

void printCanProfileCommands() {
  Serial.println("CAN profile cmd: auto detect active, 1=SantaFe override, 2=Tucson override, p=cycle profile");
}

void printCanBringupSummary() {
  const CanBackendCapabilities caps = canManager.getBackendCapabilities();
  const CanBackendRequirements reqs = canManager.getBackendRequirements();
  Serial.println("CAN backend initialized");
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
}

bool applyCanProfile(CanProfileId profileId, const char *reason) {
  const CanProfile &profile = getCanProfile(profileId);
  const CanBackendType resolvedCanBackend = profile.backendType;
  const bool useClassicCanPins =
      (AppConfig::kRequestedCanBackend == CAN_BACKEND_CLASSIC) ||
      (resolvedCanBackend == CAN_BACKEND_CLASSIC);
  const gpio_num_t canTxPin = useClassicCanPins ? AppConfig::kClassicCanTxPin : GPIO_NUM_NC;
  const gpio_num_t canRxPin = useClassicCanPins ? AppConfig::kClassicCanRxPin : GPIO_NUM_NC;

  if (reason != nullptr && reason[0] != '\0') {
    Serial.printf("CAN profile switch -> %s (%s)\n", profile.name, reason);
  }

  if (!canManager.begin(
          canTxPin,
          canRxPin,
          AppConfig::kRequestedCanBackend,
          profileId)) {
    Serial.println("CAN backend initialization failed");
    Serial.printf("CAN backend note: %s\n", canManager.getBackendDiagnosticText());
    return false;
  }

  activeCanProfile = profileId;
  fusionManager.resetCanContext();
  printCanBringupSummary();
  return true;
}

void cycleCanProfile() {
  const size_t profileCount = getCanProfileCount();
  const CanProfileId currentProfileId = canManager.isInitialized()
                                            ? canManager.getProfileId()
                                            : activeCanProfile;
  size_t activeIndex = 0;
  for (size_t i = 0; i < profileCount; ++i) {
    if (getCanProfileByIndex(i).id == currentProfileId) {
      activeIndex = i;
      break;
    }
  }

  const size_t nextIndex = (activeIndex + 1U) % profileCount;
  applyCanProfile(getCanProfileByIndex(nextIndex).id, "cycle");
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

  gpsSerial.setRxBufferSize(GPS_UART_RX_BUFFER_SIZE);
  gnss.begin(gpsSerial, GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD_RATE);
  Serial.printf("GNSS serial ready for SBF @ %lu bps\n", static_cast<unsigned long>(GPS_BAUD_RATE));
  Serial.printf(
      "GNSS UART RX buffer: %u bytes\n",
      static_cast<unsigned>(GPS_UART_RX_BUFFER_SIZE));
  if (gnssTestOverrideEnabled) {
    Serial.printf(
        "GNSS test override active: %.1f km/h, mode=%d, sats=%d\n",
        GNSS_TEST_SPEED_KMH,
        GNSS_TEST_MODE,
        GNSS_TEST_SATS);
  }
  if (extTestOverrideEnabled) {
    Serial.printf(
        "EXT test override active: %.1f km/h\n",
        EXT_TEST_SPEED_KMH);
  }

  const AppConfig::PulseInputConfig &pulseCfg = AppConfig::kPulseInputConfig;
  const float pulseMetersPerPulse = AppConfig::resolvePulseMetersPerPulse(pulseCfg.calibration);
  if (pulseInputManager.begin(
          pulseCfg.inputPin,
          pulseCfg.usePullup,
          pulseMetersPerPulse,
          pulseCfg.sampleWindowMs,
          pulseCfg.timeoutMs,
          pulseCfg.minPulseIntervalUs,
          pulseCfg.speedFilterAlpha)) {
    Serial.printf(
        "Pulse input ready: pin=%d meters/pulse=%.4f window=%lu ms timeout=%lu ms minPulse=%lu us alpha=%.2f\n",
        static_cast<int>(pulseCfg.inputPin),
        pulseMetersPerPulse,
        static_cast<unsigned long>(pulseCfg.sampleWindowMs),
        static_cast<unsigned long>(pulseCfg.timeoutMs),
        static_cast<unsigned long>(pulseCfg.minPulseIntervalUs),
        pulseCfg.speedFilterAlpha);
    if (pulseCfg.calibration.useDirectMetersPerPulse) {
      Serial.printf(
          "Pulse calibration: direct meters/pulse=%.4f\n",
          pulseCfg.calibration.directMetersPerPulse);
    } else {
      Serial.printf(
          "Pulse calibration: circumference=%.4f m, pulses/rev=%u\n",
          pulseCfg.calibration.wheelCircumferenceMeters,
          static_cast<unsigned>(pulseCfg.calibration.pulsesPerWheelRevolution));
    }
  } else {
    Serial.println("Pulse input disabled or not configured");
  }

  applyCanProfile(activeCanProfile, "boot");
  printCanProfileCommands();

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
    } else if (ch == 'e' || ch == 'E') {
      extTestOverrideEnabled = !extTestOverrideEnabled;
      Serial.printf(
          "EXT test override -> %s (%.1f km/h)\n",
          extTestOverrideEnabled ? "ON" : "OFF",
          EXT_TEST_SPEED_KMH);
    } else if (ch == '1') {
      applyCanProfile(CAN_PROFILE_SANTAFE_CLASSIC, "serial");
    } else if (ch == '2') {
      applyCanProfile(CAN_PROFILE_TUCSON_FD_CANDIDATES, "serial");
    } else if (ch == 'p' || ch == 'P') {
      cycleCanProfile();
    } else if (ch == '?' || ch == 'h' || ch == 'H') {
      printCanProfileCommands();
    }
  }

  canManager.poll(nowMs);
  const CanProfileId observedCanProfile = canManager.getProfileId();
  if (observedCanProfile != activeCanProfile) {
    activeCanProfile = observedCanProfile;
    fusionManager.resetCanContext();
  }
  pulseInputManager.update(nowMs);

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
  const PulseInputState pulseState =
      buildEffectivePulseState(pulseInputManager.getState(), nowMs);

  FusionInputs fusionInputs;
  fusionInputs.nowMs = nowMs;
  fusionInputs.gnssValid = gps.pvtValid;
  fusionInputs.gnssMode = gps.pvtMode;
  fusionInputs.gnssSatellites = gps.satellites;
  fusionInputs.gnssSpeedKmh = gps.speedKmh;
  fusionInputs.canValid = canManager.hasDecodedSpeed();
  fusionInputs.canSpeedKmh = canManager.getDecodedSpeedKmh();
  fusionInputs.extValid = pulseState.valid;
  fusionInputs.extSpeedKmh = pulseState.speedKmh;

  fusionManager.update(fusionInputs);
  distanceManager.update(nowMs, fusionManager.getState().selectedSpeedKmh);

  if (AppConfig::kEnablePeriodicGpsSummary &&
      ((nowMs - lastPrintMs) >= DEBUG_PRINT_INTERVAL_MS)) {
    lastPrintMs = nowMs;
    printGpsSummary();
  }

  if (AppConfig::kEnablePeriodicCanStatus &&
      ((nowMs - lastCanStatusMs) >= CAN_STATUS_INTERVAL_MS)) {
    lastCanStatusMs = nowMs;
    canManager.printStatus("CAN status");
    gnss.printSbfDiagnostics("GNSS SBF status");
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

