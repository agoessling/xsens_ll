#pragma once

#include <array>
#include <cstddef>

namespace xsens {

enum class BaudRate {
  k4800 = 0x0B,
  k9600 = 0x09,
  k14400 = 0x08,
  k19200 = 0x07,
  k38400 = 0x05,
  k57600 = 0x04,
  k76800 = 0x03,
  k115200 = 0x02,
  k230400 = 0x01,
  k460800 = 0x00,
  k921600 = 0x0A,
  k921600Legacy = 0x80,
  k2000000 = 0x0C,
  k3500000 = 0x0E,
  k4000000 = 0x0D,
  kInvalid = 0xFF,
};

enum class Protocol {
  kNone = 0b0000,
  kXbus = 0b0001,
  kXbusCommandOnly = 0b0010,
  kXbusDataOnly = 0b0011,
  kNmea = 0b0101,
  kRtcm = 0b0110,
};

enum class Parity {
  kNone,
  kOdd,
  kEven,
};

enum class StopBit {
  kOne,
  kTwo,
};

struct PortConfig {
  Protocol protocol;
  Parity parity;
  StopBit stop_bit;
  bool flow_control;
  BaudRate baud;
};

union PortConfigList {
  struct {
    PortConfig host_rs232;
    PortConfig host_uart;
    PortConfig rtcm_rs232;
  };
  PortConfig array[3];
};

static_assert(sizeof(PortConfigList) == 3 * sizeof(PortConfig));
static_assert(offsetof(PortConfigList, host_rs232) == offsetof(PortConfigList, array[0]));
static_assert(offsetof(PortConfigList, host_uart) == offsetof(PortConfigList, array[1]));
static_assert(offsetof(PortConfigList, rtcm_rs232) == offsetof(PortConfigList, array[2]));

struct OptionFlags {
  static OptionFlags From(uint32_t raw) {
    OptionFlags flags;

    unsigned int offset = 0;
    flags.disable_auto_store = util::UnpackField(raw, offset, 1);
    flags.disable_auto_measurement = util::UnpackField(raw, offset, 1);
    flags.enable_beidou = util::UnpackField(raw, offset, 1);
    flags.reserved0 = util::UnpackField(raw, offset, 1);
    flags.enable_ahs = util::UnpackField(raw, offset, 1);
    flags.enable_orientation_smoother = util::UnpackField(raw, offset, 1);
    flags.enable_configurable_bus_id = util::UnpackField(raw, offset, 1);
    flags.enable_inrun_compass_calibration = util::UnpackField(raw, offset, 1);
    flags.reserved1 = util::UnpackField(raw, offset, 1);
    flags.enable_config_message_at_startup = util::UnpackField(raw, offset, 1);
    flags.reserved2 = util::UnpackField(raw, offset, 1);
    flags.enable_position_velocity_smoother = util::UnpackField(raw, offset, 1);
    flags.enable_continuous_zru = util::UnpackField(raw, offset, 1);
    flags.reserved3 = util::UnpackField(raw, offset, 19);

    return flags;
  }

  uint32_t Raw() const {
    uint32_t raw = 0;

    unsigned int offset = 0;
    util::PackField(raw, disable_auto_store, offset, 1);
    util::PackField(raw, disable_auto_measurement, offset, 1);
    util::PackField(raw, enable_beidou, offset, 1);
    util::PackField(raw, reserved0, offset, 1);
    util::PackField(raw, enable_ahs, offset, 1);
    util::PackField(raw, enable_orientation_smoother, offset, 1);
    util::PackField(raw, enable_configurable_bus_id, offset, 1);
    util::PackField(raw, enable_inrun_compass_calibration, offset, 1);
    util::PackField(raw, reserved1, offset, 1);
    util::PackField(raw, enable_config_message_at_startup, offset, 1);
    util::PackField(raw, reserved2, offset, 1);
    util::PackField(raw, enable_position_velocity_smoother, offset, 1);
    util::PackField(raw, enable_continuous_zru, offset, 1);
    util::PackField(raw, reserved3, offset, 19);

    return raw;
  }

  uint32_t disable_auto_store : 1;
  uint32_t disable_auto_measurement : 1;
  uint32_t enable_beidou : 1;
  uint32_t reserved0 : 1;
  uint32_t enable_ahs : 1;
  uint32_t enable_orientation_smoother : 1;
  uint32_t enable_configurable_bus_id : 1;
  uint32_t enable_inrun_compass_calibration : 1;
  uint32_t reserved1 : 1;
  uint32_t enable_config_message_at_startup : 1;
  uint32_t reserved2 : 1;
  uint32_t enable_position_velocity_smoother : 1;
  uint32_t enable_continuous_zru : 1;
  uint32_t reserved3 : 19;
};

union Vector3f {
  struct {
    float x;
    float y;
    float z;
  };
  float array[3];
};

static_assert(sizeof(Vector3f) == 3 * sizeof(float));
static_assert(offsetof(Vector3f, x) == offsetof(Vector3f, array[0]));
static_assert(offsetof(Vector3f, y) == offsetof(Vector3f, array[1]));
static_assert(offsetof(Vector3f, z) == offsetof(Vector3f, array[2]));

union Quaternionf {
  struct {
    float w;
    float x;
    float y;
    float z;
  };
  float array[4];
};

static_assert(sizeof(Quaternionf) == 4 * sizeof(float));
static_assert(offsetof(Quaternionf, w) == offsetof(Quaternionf, array[0]));
static_assert(offsetof(Quaternionf, x) == offsetof(Quaternionf, array[1]));
static_assert(offsetof(Quaternionf, y) == offsetof(Quaternionf, array[2]));
static_assert(offsetof(Quaternionf, z) == offsetof(Quaternionf, array[3]));

enum class FilterType {
  kNone = 0,
  kMti10General = 39,
  kMti10HighMagDep = 40,
  kMti10Dynamic = 41,
  kMti10LowMagDep = 42,
  kMti10VRUGeneral = 43,
  kMti1General = 50,
  kMti1HighMagDep = 51,
  kMti1Dynamic = 52,
  kMti1NorthReference = 53,
  kMti1VRUGeneral = 54,
  kMti7General = 11,
  kMti7GeneralNoBaro = 12,
  kMti7GeneralMag = 13,
  kMti8GeneralRtk = 14,
  kMti8GeneralNoBaroRtk = 15,
  kMti8GeneralMagRtk = 16,
  Mti710General = 1,
  Mti710GeneralNoBaro = 2,
  Mti710GeneralMag = 3,
  Mti710Automotive = 4,
  Mti710HighPerformanceEdr = 9,
};

struct FilterProfile {
  FilterType type;
  uint8_t version;
  char label[20];
};

enum class GnssPlatform {
  kPortable = 0,
  kStationary = 2,
  kPedestrian = 3,
  kAutomotive = 4,
  kAtSea = 5,
  kAirborneLt1g = 6,
  kAirborneLt2g = 7,
  kAirborneLt4g = 8,
  kWrist = 9,
  kBike = 10,
};

enum class GnssType {
  kUbloxMaxM8q = 0,
  kNmea = 1,
  kUbloxNeoM8p = 2,
  kUbloxZedF9p = 3,
  kSeptentrioMosiacX5 = 4,
  kTrimbleBx992 = 5,
};

enum class GnssNmeaOptions {
  kGl = 0,
  kGn = 1,
  kGp = 2,
};

union GnssOptions {
  GnssPlatform ublox;
  GnssNmeaOptions nmea;
  uint32_t raw;
};

static_assert(sizeof(GnssOptions) == 4);
static_assert(sizeof(GnssPlatform) == 4);
static_assert(sizeof(GnssNmeaOptions) == 4);
static_assert(offsetof(GnssOptions, ublox) == offsetof(GnssOptions, nmea));
static_assert(offsetof(GnssOptions, ublox) == offsetof(GnssOptions, raw));

struct GnssSettings {
  GnssType type;
  BaudRate baud;
  uint16_t msg_rate;
  GnssOptions options;
};

struct FirmwareRev {
  uint8_t major;
  uint8_t minor;
  uint8_t rev;
  uint32_t build_number;
  uint32_t scm_reference;
};

struct OutputConfig {
  uint16_t type_id;
  uint16_t rate;
};

};  // namespace xsens
