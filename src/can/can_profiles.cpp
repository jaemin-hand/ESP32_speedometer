#include "can_profiles.h"

namespace {

constexpr CanSpeedDecoderConfig kSantaFeClassicDecoders[] = {
    {
        true,                       // bool enabled
        "santafe_replay_speed",     // const char *name
        0x450,                      // uint32_t identifier = 0;
        false,                      // bool extended = false;
        false,                      // bool fdFormat = false;
        CAN_SPEED_DECODER_UNSIGNED, // CanSpeedDecoderType decoderType = CAN_SPEED_DECODER_UNSIGNED;
        10,                         // uint8_t priority = 0;
        0,                          // uint8_t startByte = 0;
        1,                          // uint8_t lengthBytes = 2;
        CAN_SIGNAL_LITTLE_ENDIAN,   // CanSignalEndian endian = CAN_SIGNAL_LITTLE_ENDIAN;
        1.0f,                       // float scale = 1.0f;
        0.0f,                       // float offset = 0.0f;
        250,                        // uint32_t timeoutMs = 200;
    },
    {
        true,
        "santafe_wheel_avg_0x386",
        0x386,
        false,
        false,
        CAN_SPEED_DECODER_SANTAFE_WHEEL_AVG_0X386,
        20,
        0,
        8,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f,
        0.0f,
        250,
    },
};

// Tucson NX4 CAN-FD speed candidates from offline log analysis.
// Best decimal-resolution candidates:
//   - 0x0B5 bytes[22..23] little-endian * 0.01
//   - 0x040 bytes[16..17] little-endian * 0.01
// Coarse fallback only:
//   - 0x145 bytes[6..7] little-endian / 256.0
constexpr CanSpeedDecoderConfig kTucsonFdCandidateDecoders[] = {
    {
        true,
        "tucson_fd_vehicle_speed_0x0b5",
        0x0B5,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        30,
        22,
        2,
        CAN_SIGNAL_LITTLE_ENDIAN,
        0.01f,
        0.0f,
        250,
    },
    {
        false,
        "tucson_fd_vehicle_speed_0x040",
        0x040,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        25,
        16,
        2,
        CAN_SIGNAL_LITTLE_ENDIAN,
        0.01f,
        0.0f,
        250,
    },
    {
        false,
        "tucson_fd_coarse_fallback_0x145",
        0x145,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        10,
        6,
        2,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f / 256.0f,
        0.0f,
        250,
    },
};

constexpr CanProfile kProfiles[] = {
    {
        CAN_PROFILE_SANTAFE_CLASSIC,
        "SantaFe Classic CAN",
        CAN_BACKEND_FD,
        "Unified MCP2517FD path: classic CAN decoders (0x450, 0x386) now run through the external controller at 500 kbps nominal",
        kSantaFeClassicDecoders,
        sizeof(kSantaFeClassicDecoders) / sizeof(kSantaFeClassicDecoders[0]),
    },
    {
        CAN_PROFILE_TUCSON_FD_CANDIDATES,
        "Tucson NX4 CAN-FD Candidates",
        CAN_BACKEND_FD,
        "Single-source test: only 0x0B5 u16@22 /100 enabled; 0x040 and 0x145 disabled to isolate source mixing",
        kTucsonFdCandidateDecoders,
        sizeof(kTucsonFdCandidateDecoders) / sizeof(kTucsonFdCandidateDecoders[0]),
    },
};

}  // namespace

size_t getCanProfileCount() {
  return sizeof(kProfiles) / sizeof(kProfiles[0]);
}

const CanProfile &getCanProfileByIndex(size_t index) {
  if (index < getCanProfileCount()) {
    return kProfiles[index];
  }

  return kProfiles[0];
}

const CanProfile &getCanProfile(CanProfileId profileId) {
  for (const CanProfile &profile : kProfiles) {
    if (profile.id == profileId) {
      return profile;
    }
  }

  return kProfiles[0];
}
