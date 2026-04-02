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

// Tucson NX4 candidate list from offline CAN-FD log analysis.
// These remain disabled until the CAN-FD backend is implemented and the
// payload interpretation is verified against live data.
constexpr CanSpeedDecoderConfig kTucsonFdCandidateDecoders[] = {
    {
        false,
        "tucson_fd_candidate_0x100_b20",
        0x100,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        10,
        20,
        1,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f,
        0.0f,
        250,
    },
    {
        false,
        "tucson_fd_candidate_0x145_b07",
        0x145,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        10,
        7,
        1,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f,
        0.0f,
        250,
    },
    {
        false,
        "tucson_fd_candidate_0x0B5_b08",
        0x0B5,
        false,
        true,
        CAN_SPEED_DECODER_UNSIGNED,
        10,
        8,
        1,
        CAN_SIGNAL_LITTLE_ENDIAN,
        1.0f,
        0.0f,
        250,
    },
};

constexpr CanProfile kProfiles[] = {
    {
        CAN_PROFILE_SANTAFE_CLASSIC,
        "SantaFe Classic CAN",
        CAN_BACKEND_CLASSIC,
        "Working baseline: raw monitor + 0x450 byte0 fallback + 0x386 wheel-average candidate",
        kSantaFeClassicDecoders,
        sizeof(kSantaFeClassicDecoders) / sizeof(kSantaFeClassicDecoders[0]),
    },
    {
        CAN_PROFILE_TUCSON_FD_CANDIDATES,
        "Tucson NX4 CAN-FD Candidates",
        CAN_BACKEND_FD,
        "Offline FD candidates only for now: 0x100 byte20, 0x145 byte7, 0x0B5 byte8",
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
