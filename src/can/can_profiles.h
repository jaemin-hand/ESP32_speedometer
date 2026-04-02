#pragma once

#include <stddef.h>
#include <stdint.h>

#include "can_backend.h"

enum CanSignalEndian : uint8_t {
  CAN_SIGNAL_LITTLE_ENDIAN = 0,
  CAN_SIGNAL_BIG_ENDIAN = 1,
};

enum CanSpeedDecoderType : uint8_t {
  CAN_SPEED_DECODER_UNSIGNED = 0,
  CAN_SPEED_DECODER_SANTAFE_WHEEL_AVG_0X386 = 1,
};

struct CanSpeedDecoderConfig {
  bool enabled = false;
  const char *name = "";
  uint32_t identifier = 0;
  bool extended = false;
  bool fdFormat = false;
  CanSpeedDecoderType decoderType = CAN_SPEED_DECODER_UNSIGNED;
  uint8_t priority = 0;
  uint8_t startByte = 0;
  uint8_t lengthBytes = 2;
  CanSignalEndian endian = CAN_SIGNAL_LITTLE_ENDIAN;
  float scale = 1.0f;
  float offset = 0.0f;
  uint32_t timeoutMs = 200;
};

enum CanProfileId : uint8_t {
  CAN_PROFILE_SANTAFE_CLASSIC = 0,
  CAN_PROFILE_TUCSON_FD_CANDIDATES = 1,
};

struct CanProfile {
  CanProfileId id = CAN_PROFILE_SANTAFE_CLASSIC;
  const char *name = "";
  CanBackendType backendType = CAN_BACKEND_CLASSIC;
  const char *bringupNote = "";
  const CanSpeedDecoderConfig *speedDecoders = nullptr;
  size_t speedDecoderCount = 0;
};

size_t getCanProfileCount();
const CanProfile &getCanProfileByIndex(size_t index);
const CanProfile &getCanProfile(CanProfileId profileId);
