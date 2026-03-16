#include "gnss_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr uint8_t SBF_SYNC_1 = 0x24;
constexpr uint8_t SBF_SYNC_2 = 0x40;
constexpr uint16_t SBF_BLOCK_PVT_GEODETIC = 4007;
constexpr uint16_t SBF_BLOCK_RECEIVER_TIME = 5914;
constexpr size_t SBF_BLOCK_BUFFER_SIZE = 512;
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
}

void GnssManager::update() {
  if (serial_ == nullptr) {
    return;
  }

  while (serial_->available()) {
    processSbfByte(static_cast<uint8_t>(serial_->read()));
  }
}

const GpsData &GnssManager::getData() const {
  return gps_;
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

void GnssManager::handleSbfBlock(const uint8_t *block, uint16_t length) {
  if (length < 8) {
    return;
  }

  const uint16_t blockId = readLittleEndian<uint16_t>(block + 4);
  const uint16_t blockNumber = blockId & 0x1FFFU;

  switch (blockNumber) {
    case SBF_BLOCK_PVT_GEODETIC:
      parsePvtGeodeticBlock(block, length);
      break;
    case SBF_BLOCK_RECEIVER_TIME:
      parseReceiverTimeBlock(block, length);
      break;
    default:
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
      resetDecoder();
      return;
    }
  }

  if (sbfExpectedLength_ != 0 && sbfBlockSize_ == sbfExpectedLength_) {
    const uint16_t receivedCrc = readLittleEndian<uint16_t>(sbfBlockBuffer_ + 2);
    const uint16_t computedCrc = computeSbfCrc(sbfBlockBuffer_ + 4, sbfExpectedLength_ - 4);

    if (receivedCrc == computedCrc) {
      handleSbfBlock(sbfBlockBuffer_, static_cast<uint16_t>(sbfExpectedLength_));
    }

    resetDecoder();
  }
}
