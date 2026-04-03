#include "gnss_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr uint8_t SBF_SYNC_1 = 0x24; // $ ASCII
constexpr uint8_t SBF_SYNC_2 = 0x40; // @ ASCII <- Notify the start point when these two are continuous
constexpr uint16_t SBF_BLOCK_PVT_GEODETIC = 4007; // ID that informs you that the box contains the PVT value
constexpr uint16_t SBF_BLOCK_MEAS_EPOCH = 4027;
constexpr uint16_t SBF_BLOCK_RECEIVER_TIME = 5914; // ID that informs you that it is a box containing the time to receive it
constexpr size_t SBF_BLOCK_BUFFER_SIZE = 4096; // MeasEpoch can exceed 512 bytes when many signals are tracked.
constexpr uint32_t GNSS_PVT_TIMEOUT_MS = 300;
constexpr uint32_t GNSS_CN0_TIMEOUT_MS = 1500;
constexpr uint32_t GNSS_SBF_STREAM_TIMEOUT_MS = 1000;
constexpr uint32_t GNSS_RECEIVER_TIME_TIMEOUT_MS = 1500;
constexpr double SBF_DNU_THRESHOLD = -1.99e10;
constexpr double RAD_TO_DEGREES = 57.29577951308232;
constexpr float MPS_TO_KNOTS = 1.9438445f;

template <typename T>
T readLittleEndian(const uint8_t *data) {
  T value;
  memcpy(&value, data, sizeof(T));
  return value;
}

uint16_t computeSbfCrc(const uint8_t *data, size_t length) {
  uint16_t crc = 0;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

bool isSbfDoubleValid(double value) {
  return isfinite(value) && value > SBF_DNU_THRESHOLD;
}

bool isSbfFloatValid(float value) {
  return isfinite(value) && value > static_cast<float>(SBF_DNU_THRESHOLD);
}

}  // namespace

void GnssManager::begin(HardwareSerial &serial, int rxPin, int txPin, uint32_t baudRate) {
  serial_ = &serial;
  serial_->begin(baudRate, SERIAL_8N1, rxPin, txPin);
  gps_ = GpsData{};
  lastSbfByteMs_ = 0;
  lastPvtUpdateMs_ = 0;
  lastMeasEpochUpdateMs_ = 0;
  lastReceiverTimeUpdateMs_ = 0;
  sbfPvtBlockCount_ = 0;
  sbfReceiverTimeBlockCount_ = 0;
  sbfMeasEpochBlockCount_ = 0;
  sbfUnknownBlockCount_ = 0;
  sbfCrcRejectCount_ = 0;
  sbfLengthRejectCount_ = 0;
  lastSbfBlockNumber_ = 0;
  lastSbfBlockLength_ = 0;
  resetDecoder();
}

void GnssManager::update() {
  if (serial_ == nullptr) {
    return;
  }

  while (serial_->available()) {
    lastSbfByteMs_ = millis();
    processSbfByte(static_cast<uint8_t>(serial_->read()));
  }

  const uint32_t nowMs = millis();
  gps_.sbfAgeMs = (lastSbfByteMs_ != 0U) ? (nowMs - lastSbfByteMs_) : UINT32_MAX;
  gps_.pvtAgeMs = (lastPvtUpdateMs_ != 0U) ? (nowMs - lastPvtUpdateMs_) : UINT32_MAX;
  gps_.cn0AgeMs =
      (lastMeasEpochUpdateMs_ != 0U) ? (nowMs - lastMeasEpochUpdateMs_) : UINT32_MAX;
  gps_.receiverTimeAgeMs =
      (lastReceiverTimeUpdateMs_ != 0U) ? (nowMs - lastReceiverTimeUpdateMs_) : UINT32_MAX;
  gps_.sbfStreamActive =
      (lastSbfByteMs_ != 0U) && (gps_.sbfAgeMs <= GNSS_SBF_STREAM_TIMEOUT_MS);
  gps_.receiverTimeValid =
      (lastReceiverTimeUpdateMs_ != 0U) &&
      (gps_.receiverTimeAgeMs <= GNSS_RECEIVER_TIME_TIMEOUT_MS) &&
      (strlen(gps_.timeStr) >= 8U);

  if (gps_.pvtValid && (lastPvtUpdateMs_ != 0U)) {
    if ((nowMs - lastPvtUpdateMs_) > GNSS_PVT_TIMEOUT_MS) {
      gps_.pvtValid = false;
      gps_.pvtMode = 0;
      gps_.errorCode = 0;
      gps_.speedKnots = 0.0f;
      gps_.speedKmh = 0.0f;
    }
  }

  if (gps_.cn0Valid && (lastMeasEpochUpdateMs_ != 0U)) {
    if ((nowMs - lastMeasEpochUpdateMs_) > GNSS_CN0_TIMEOUT_MS) {
      gps_.cn0Valid = false;
      gps_.cn0SignalCount = 0;
      gps_.cn0AvgDbHz = 0.0f;
      gps_.cn0MaxDbHz = 0.0f;
    }
  }
}

const GpsData &GnssManager::getData() const {
  return gps_;
}

void GnssManager::printSbfDiagnostics(const char *label) const {
  if (label != nullptr) {
    Serial.println(label);
  }
  Serial.printf(
      "SBF diag: last_block=%u len=%u pvt=%lu time=%lu meas=%lu unknown=%lu crc_drop=%lu len_drop=%lu\n",
      static_cast<unsigned>(lastSbfBlockNumber_),
      static_cast<unsigned>(lastSbfBlockLength_),
      static_cast<unsigned long>(sbfPvtBlockCount_),
      static_cast<unsigned long>(sbfReceiverTimeBlockCount_),
      static_cast<unsigned long>(sbfMeasEpochBlockCount_),
      static_cast<unsigned long>(sbfUnknownBlockCount_),
      static_cast<unsigned long>(sbfCrcRejectCount_),
      static_cast<unsigned long>(sbfLengthRejectCount_));
}

const char *GnssManager::pvtStatusToText(int pvtMode, int errorCode) {
  if (errorCode != 0 || pvtMode == 0) {
    switch (errorCode) {
      case 1: return "NO MEAS";
      case 2: return "NO EPH";
      case 3: return "DOP>15";
      case 4: return "RESIDUAL";
      case 5: return "NO CONV";
      case 6: return "OUTLIER";
      case 7: return "EXPORT";
      case 8: return "NO CORR";
      case 9: return "NO BASE";
      case 10: return "WAIT FIX";
      default: return "NO PVT";
    }
  }

  switch (pvtMode) {
    case 1: return "SPP";
    case 2: return "DGPS";
    case 3: return "FIXED";
    case 4: return "RTK FIX";
    case 5: return "RTK FLOAT";
    case 6: return "SBAS";
    case 7: return "MB RTK FIX";
    case 8: return "MB RTK FLT";
    case 10: return "PPP";
    default: return "OTHER";
  }
}

void GnssManager::parsePvtGeodeticBlock(const uint8_t *block, uint16_t length) {
  if (length < 75) {
    return;
  }

  lastPvtUpdateMs_ = millis();

  gps_.pvtMode = block[14] & 0x0F;
  gps_.errorCode = block[15];
  gps_.pvtValid = (gps_.errorCode == 0 && gps_.pvtMode != 0);
  gps_.satellites = block[74];

  if (!gps_.pvtValid) {
    gps_.speedKnots = 0.0f;
    gps_.speedKmh = 0.0f;
    return;
  }

  const double latitudeRad = readLittleEndian<double>(block + 16);
  const double longitudeRad = readLittleEndian<double>(block + 24);
  const double heightM = readLittleEndian<double>(block + 32);
  const float undulationM = readLittleEndian<float>(block + 40);
  const float velocityNorth = readLittleEndian<float>(block + 44);
  const float velocityEast = readLittleEndian<float>(block + 48);

  if (isSbfDoubleValid(latitudeRad)) {
    gps_.latitude = latitudeRad * RAD_TO_DEGREES;
  }

  if (isSbfDoubleValid(longitudeRad)) {
    gps_.longitude = longitudeRad * RAD_TO_DEGREES;
  }

  if (isSbfDoubleValid(heightM) && isSbfFloatValid(undulationM)) {
    gps_.altitudeM = static_cast<float>(heightM - undulationM);
  } else if (isSbfDoubleValid(heightM)) {
    gps_.altitudeM = static_cast<float>(heightM);
  }

  if (isSbfFloatValid(velocityNorth) && isSbfFloatValid(velocityEast)) {
    const float speedMps = sqrtf((velocityNorth * velocityNorth) + (velocityEast * velocityEast));
    gps_.speedKmh = speedMps * 3.6f;
    gps_.speedKnots = speedMps * MPS_TO_KNOTS;
  }
}

void GnssManager::parseReceiverTimeBlock(const uint8_t *block, uint16_t length) {
  if (length < 22) {
    return;
  }

  lastReceiverTimeUpdateMs_ = millis();

  const int8_t utcYear = static_cast<int8_t>(block[14]);
  const int8_t utcMonth = static_cast<int8_t>(block[15]);
  const int8_t utcDay = static_cast<int8_t>(block[16]);
  const int8_t utcHour = static_cast<int8_t>(block[17]);
  const int8_t utcMinute = static_cast<int8_t>(block[18]);
  const int8_t utcSecond = static_cast<int8_t>(block[19]);
  const uint8_t syncLevel = block[21];

  const bool utcValid =
      ((syncLevel & 0x07U) == 0x07U) &&
      utcYear >= 0 &&
      utcMonth >= 1 && utcMonth <= 12 &&
      utcDay >= 1 && utcDay <= 31 &&
      utcHour >= 0 && utcHour <= 23 &&
      utcMinute >= 0 && utcMinute <= 59 &&
      utcSecond >= 0 && utcSecond <= 59;

  if (!utcValid) {
    return;
  }

  snprintf(gps_.dateStr, sizeof(gps_.dateStr), "20%02d-%02d-%02d", utcYear, utcMonth, utcDay);
  snprintf(gps_.timeStr, sizeof(gps_.timeStr), "%02d:%02d:%02d", utcHour, utcMinute, utcSecond);
}

void GnssManager::parseMeasEpochBlock(const uint8_t *block, uint16_t length) {
  if (length < 20) {
    return;
  }

  const uint8_t signalCount = block[14];
  const uint8_t type1Length = block[15];
  const uint8_t type2Length = block[16];
  if (signalCount == 0U || type1Length < 20U) {
    return;
  }

  lastMeasEpochUpdateMs_ = millis();

  size_t offset = 20;
  float cn0SumDbHz = 0.0f;
  float cn0MaxDbHz = 0.0f;
  uint16_t cn0Count = 0;

  for (uint8_t i = 0; i < signalCount; ++i) {
    if ((offset + type1Length) > length) {
      break;
    }

    const uint8_t cn0Raw = block[offset + 15];
    const uint8_t n2 = block[offset + 19];
    if (cn0Raw != 255U) {
      const float cn0DbHz = static_cast<float>(cn0Raw) * 0.25f;
      if (cn0DbHz > 0.0f && cn0DbHz < 80.0f) {
        cn0SumDbHz += cn0DbHz;
        if (cn0DbHz > cn0MaxDbHz) {
          cn0MaxDbHz = cn0DbHz;
        }
        ++cn0Count;
      }
    }

    const size_t nextOffset =
        offset + type1Length + (static_cast<size_t>(n2) * type2Length);
    if (nextOffset <= offset) {
      break;
    }
    offset = nextOffset;
  }

  if (cn0Count == 0U) {
    gps_.cn0Valid = false;
    gps_.cn0SignalCount = 0;
    gps_.cn0AvgDbHz = 0.0f;
    gps_.cn0MaxDbHz = 0.0f;
    return;
  }

  gps_.cn0Valid = true;
  gps_.cn0SignalCount = static_cast<uint8_t>(cn0Count > 255U ? 255U : cn0Count);
  gps_.cn0AvgDbHz = cn0SumDbHz / static_cast<float>(cn0Count);
  gps_.cn0MaxDbHz = cn0MaxDbHz;
}

void GnssManager::handleSbfBlock(const uint8_t *block, uint16_t length) {
  if (length < 8) {
    return;
  }

  const uint16_t blockId = readLittleEndian<uint16_t>(block + 4);
  const uint16_t blockNumber = blockId & 0x1FFFU;
  lastSbfBlockNumber_ = blockNumber;
  lastSbfBlockLength_ = length;

  switch (blockNumber) {
    case SBF_BLOCK_PVT_GEODETIC:
      ++sbfPvtBlockCount_;
      parsePvtGeodeticBlock(block, length);
      break;
    case SBF_BLOCK_MEAS_EPOCH:
      ++sbfMeasEpochBlockCount_;
      parseMeasEpochBlock(block, length);
      break;
    case SBF_BLOCK_RECEIVER_TIME:
      ++sbfReceiverTimeBlockCount_;
      parseReceiverTimeBlock(block, length);
      break;
    default:
      ++sbfUnknownBlockCount_;
      break;
  }
}

void GnssManager::resetDecoder() {
  sbfBlockSize_ = 0;
  sbfExpectedLength_ = 0;
}

void GnssManager::processSbfByte(uint8_t byteValue) {
  if (sbfBlockSize_ == 0) {
    if (byteValue == SBF_SYNC_1) {
      sbfBlockBuffer_[sbfBlockSize_++] = byteValue;
    }
    return;
  }

  if (sbfBlockSize_ == 1) {
    if (byteValue == SBF_SYNC_2) {
      sbfBlockBuffer_[sbfBlockSize_++] = byteValue;
    } else if (byteValue == SBF_SYNC_1) {
      sbfBlockBuffer_[0] = byteValue;
      sbfBlockSize_ = 1;
    } else {
      resetDecoder();
    }
    return;
  }

  if (sbfBlockSize_ >= SBF_BLOCK_BUFFER_SIZE) {
    resetDecoder();
    return;
  }

  sbfBlockBuffer_[sbfBlockSize_++] = byteValue;

  if (sbfBlockSize_ == 8) {
    sbfExpectedLength_ = readLittleEndian<uint16_t>(sbfBlockBuffer_ + 6);
    if (sbfExpectedLength_ < 8 ||
        (sbfExpectedLength_ % 4) != 0 ||
        sbfExpectedLength_ > SBF_BLOCK_BUFFER_SIZE) {
      ++sbfLengthRejectCount_;
      resetDecoder();
      return;
    }
  }

  if (sbfExpectedLength_ != 0 && sbfBlockSize_ == sbfExpectedLength_) {
    const uint16_t receivedCrc = readLittleEndian<uint16_t>(sbfBlockBuffer_ + 2);
    const uint16_t computedCrc = computeSbfCrc(sbfBlockBuffer_ + 4, sbfExpectedLength_ - 4);

    if (receivedCrc == computedCrc) {
      handleSbfBlock(sbfBlockBuffer_, static_cast<uint16_t>(sbfExpectedLength_));
    } else {
      ++sbfCrcRejectCount_;
    }

    resetDecoder();
  }
}
