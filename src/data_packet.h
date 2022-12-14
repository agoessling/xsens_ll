#pragma once

#include <cstdint>
#include <optional>

#include "src/bitfield.h"
#include "src/xbus_parser.h"

namespace xsens {
namespace data {

enum class TypeId {
  kTemperature = 0x0810,  // [°C] Temperature
  kUtcTime = 0x1010,  // [] UTC Time
  kPacketCounter = 0x1020,  // [] Packet Counter
  kSampleTimeFine = 0x1060,  // [] Sample Time Fine
  kSampleTimeCoarse = 0x1070,  // [s] Sample Time Coarse
  kQuaternion = 0x2010,  // [] Quaternion
  kRotationMatrix = 0x2020,  // [] Rotation Matrix
  kEulerAngles = 0x2030,  // [deg] Euler Angles
  kBaroPressure = 0x3010,  // [Pa] Baro Pressure
  kDeltaV = 0x4010,  // [m/s] Delta V
  kAcceleration = 0x4020,  // [m/s2] Acceleration
  kFreeAcceleration = 0x4030,  // [m/s2] Free Acceleration
  kAccelerationHr = 0x4040,  // [m/s2] AccelerationHR
  kAltitudeEllipsoid = 0x5020,  // [m] Altitude Ellipsoid
  kPositionEcef = 0x5030,  // [m] Position ECEF
  kLatLon = 0x5040,  // [deg] LatLon
  kGnssPvtData = 0x7010,  // [] GNSS PVT data
  kGnssSatInfo = 0x7020,  // [] GNSS satellites info
  kGnssPvtPulse = 0x7030,  // [s] GNSS PVT pulse
  kRateOfTurn = 0x8020,  // [rad/s] Rate of Turn
  kDeltaQ = 0x8030,  // [] Delta Q
  kRateOfTurnHr = 0x8040,  // [rad/s] RateOfTurnHR
  kRawAccGyrMagTemp = 0xA010,  // [] ACC, GYR, MAG, temperature
  kRawGyroTemp = 0xA020,  // [°C] Gyro temperatures
  kMagneticField = 0xC020,  // [a.u.] Magnetic Field
  kVelocity = 0xD010,  // [m/s] Velocity XYZ
  kStatusByte = 0xE010,  // [] Status Byte
  kStatusWord = 0xE020,  // [] Status Word
  kDeviceId = 0xE080,  // [] Device ID
  kLocationId = 0xE090,  // [] Location ID
};

enum class PrecisionId {
  kU1 = 0x00,
  kU2 = 0x00,
  kU4 = 0x00,
  kU8 = 0x00,
  kI1 = 0x00,
  kI2 = 0x00,
  kI4 = 0x00,
  kI8 = 0x00,
  kFloat32 = 0x00,
  kFp1220 = 0x01,
  kFp1632 = 0x02,
  kFloat64 = 0x03,
};

enum class CoordinateSystemId {
  kSensor = 0x00,
  kEnu = 0x00,
  kNed = 0x04,
  kNwu = 0x08,
};

enum class RtkSolution {
  kNone = 0x00,
  kFloating = 0x01,
  kFixed = 0x02,
};

struct U1 {
  using type = uint8_t;
  static constexpr PrecisionId id = PrecisionId::kU1;
  static constexpr unsigned int size = 1;
};

struct U2 {
  using type = uint16_t;
  static constexpr PrecisionId id = PrecisionId::kU2;
  static constexpr unsigned int size = 2;
};

struct U4 {
  using type = uint32_t;
  static constexpr PrecisionId id = PrecisionId::kU4;
  static constexpr unsigned int size = 4;
};

struct U8 {
  using type = uint64_t;
  static constexpr PrecisionId id = PrecisionId::kU8;
  static constexpr unsigned int size = 8;
};

struct I1 {
  using type = int8_t;
  static constexpr PrecisionId id = PrecisionId::kI1;
  static constexpr unsigned int size = 1;
};

struct I2 {
  using type = int16_t;
  static constexpr PrecisionId id = PrecisionId::kI2;
  static constexpr unsigned int size = 2;
};

struct I4 {
  using type = int32_t;
  static constexpr PrecisionId id = PrecisionId::kI4;
  static constexpr unsigned int size = 4;
};

struct I8 {
  using type = int64_t;
  static constexpr PrecisionId id = PrecisionId::kI8;
  static constexpr unsigned int size = 8;
};

struct Float32 {
  using type = float;
  static constexpr PrecisionId id = PrecisionId::kFloat32;
  static constexpr unsigned int size = 4;
};

struct Fp1220 {
  using type = int32_t;
  static constexpr PrecisionId id = PrecisionId::kFp1220;
  static constexpr unsigned int size = 4;
};

struct Fp1632 {
  using type = int64_t;
  static constexpr PrecisionId id = PrecisionId::kFp1632;
  static constexpr unsigned int size = 6;
};

struct Float64 {
  using type = double;
  static constexpr PrecisionId id = PrecisionId::kFloat64;
  static constexpr unsigned int size = 8;
};

struct Sensor {
  static constexpr CoordinateSystemId id = CoordinateSystemId::kSensor;
};

struct Enu {
  static constexpr CoordinateSystemId id = CoordinateSystemId::kEnu;
};

struct Ned {
  static constexpr CoordinateSystemId id = CoordinateSystemId::kNed;
};

struct Nwu {
  static constexpr CoordinateSystemId id = CoordinateSystemId::kNwu;
};

static constexpr uint16_t GetDataId(TypeId data_id, PrecisionId precision_id,
                                    CoordinateSystemId coords_id) {
  return (static_cast<uint16_t>(data_id) | static_cast<uint16_t>(precision_id) |
          static_cast<uint16_t>(coords_id));
}

static constexpr TypeId GetTypeId(uint16_t id) {
  return static_cast<TypeId>(id & 0xFFF0);
}

static constexpr PrecisionId GetPrecisionId(uint16_t id) {
  return static_cast<PrecisionId>(id & 0x03);
}

static constexpr CoordinateSystemId GetCoordinateSystemId(uint16_t id) {
  return static_cast<CoordinateSystemId>(id & 0x0C);
}

template <typename Precision>
unsigned int UnpackPrimitive(typename Precision::type& data, const uint8_t *buf) {
  if constexpr (std::is_same<Precision, U1>::value) {
    data = static_cast<typename Precision::type>(buf[0]);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, U2>::value) {
    data = UnpackBigEndian16<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, U4>::value) {
    data = UnpackBigEndian32<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, U8>::value) {
    data = UnpackBigEndian64<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, I1>::value) {
    data = static_cast<typename Precision::type>(buf[0]);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, I2>::value) {
    data = UnpackBigEndian16<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, I4>::value) {
    data = UnpackBigEndian32<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, I8>::value) {
    data = UnpackBigEndian64<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, Float32>::value) {
    data = UnpackBigEndian32<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, Fp1220>::value) {
    data = UnpackBigEndian32<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, Fp1632>::value) {
    data = UnpackBigEndian48<typename Precision::type>(buf);
    return Precision::size;
  }
  if constexpr (std::is_same<Precision, Float64>::value) {
    data = UnpackBigEndian64<typename Precision::type>(buf);
    return Precision::size;
  }
  // No return will cause compilation failure if no constexpr if matches.
}

template <typename Derived, TypeId type_id, typename Precision>
struct Scalar {
  static constexpr uint16_t id = GetDataId(type_id, Precision::id, CoordinateSystemId::kSensor);

  static std::optional<Derived> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != Precision::size) {
      return std::nullopt;
    }

    Derived data;
    UnpackPrimitive<Precision>(data.val, buf);
    return data;
  }

  typename Precision::type val;
};

template <typename Precision>
struct Temperature : Scalar<Temperature<Precision>, TypeId::kTemperature, Precision> {};

template <typename Precision>
struct AltitudeEllipsoid
    : Scalar<AltitudeEllipsoid<Precision>, TypeId::kAltitudeEllipsoid, Precision> {};

struct PacketCounter : Scalar<PacketCounter, TypeId::kPacketCounter, U2> {};
struct SampleTimeFine : Scalar<SampleTimeFine, TypeId::kSampleTimeFine, U4> {};
struct SampleTimeCoarse : Scalar<SampleTimeCoarse, TypeId::kSampleTimeCoarse, U4> {};
struct BaroPressure : Scalar<BaroPressure, TypeId::kBaroPressure, U4> {};
struct GnssPvtPulse : Scalar<GnssPvtPulse, TypeId::kGnssPvtPulse, U4> {};
struct DeviceIdU4 : Scalar<DeviceIdU4, TypeId::kDeviceId, U4> {};
struct DeviceIdU8 : Scalar<DeviceIdU8, TypeId::kDeviceId, U8> {};
struct LocationId : Scalar<LocationId, TypeId::kLocationId, U2> {};

// Struct Base Data Types.
struct StatusByte {
  static const uint16_t id = static_cast<uint16_t>(TypeId::kStatusByte);

  enum class NoRotationStatus {
    kRunning = 0x03,
    kRotationDetected = 0x02,
    kComplete = 0x00,
  };

  NoRotationStatus no_rotation_status() const {
    return static_cast<NoRotationStatus>(no_rotation_status_raw);
  }

  static std::optional<StatusByte> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != 1) return std::nullopt;

    StatusByte status;
    const uint8_t raw = buf[0];

    unsigned int offset = 0;
    status.self_test = util::UnpackField(raw, offset, 1);
    status.filter_valid = util::UnpackField(raw, offset, 1);
    status.gnss_fix = util::UnpackField(raw, offset, 1);
    status.no_rotation_status_raw = util::UnpackField(raw, offset, 2);
    status.rep_mo = util::UnpackField(raw, offset, 1);
    status.clock_sync = util::UnpackField(raw, offset, 1);
    status.reserved1 = util::UnpackField(raw, offset, 1);

    return status;
  }

  uint8_t self_test : 1;
  uint8_t filter_valid : 1;
  uint8_t gnss_fix : 1;
  uint8_t no_rotation_status_raw : 2;
  uint8_t rep_mo : 1;
  uint8_t clock_sync : 1;
  uint8_t reserved1 : 1;
};

struct StatusWord {
  static const uint16_t id = static_cast<uint16_t>(TypeId::kStatusWord);

  enum class FilterMode {
    kNoGnss = 0x00,
    kCoasting = 0x01,
    kWithGnss = 0x03,
  };

  FilterMode filter_mode() const { return static_cast<FilterMode>(filter_mode_raw); }

  RtkSolution rtk_status() const { return static_cast<RtkSolution>(rtk_status_raw); }

  static std::optional<StatusWord> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != 4) return std::nullopt;
    const uint32_t raw = UnpackBigEndian32<uint32_t>(buf);

    StatusWord status;
    status.status_byte = *StatusByte::Unpack(buf + 3, 1);

    unsigned int offset = 8;
    status.clip_accel_x = util::UnpackField(raw, offset, 1);
    status.clip_accel_y = util::UnpackField(raw, offset, 1);
    status.clip_accel_z = util::UnpackField(raw, offset, 1);
    status.clip_gyro_x = util::UnpackField(raw, offset, 1);
    status.clip_gyro_y = util::UnpackField(raw, offset, 1);
    status.clip_gyro_z = util::UnpackField(raw, offset, 1);
    status.clip_mag_x = util::UnpackField(raw, offset, 1);
    status.clip_mag_y = util::UnpackField(raw, offset, 1);
    status.clip_mag_z = util::UnpackField(raw, offset, 1);
    status.reserved2 = util::UnpackField(raw, offset, 2);
    status.clipping = util::UnpackField(raw, offset, 1);
    status.reserved3 = util::UnpackField(raw, offset, 1);
    status.sync_in = util::UnpackField(raw, offset, 1);
    status.sync_out = util::UnpackField(raw, offset, 1);
    status.filter_mode_raw = util::UnpackField(raw, offset, 3);
    status.gnss_time_pulse = util::UnpackField(raw, offset, 1);
    status.rtk_status_raw = util::UnpackField(raw, offset, 2);
    status.reserved4 = util::UnpackField(raw, offset, 3);

    return status;
  }

  StatusByte status_byte;
  uint32_t clip_accel_x : 1;
  uint32_t clip_accel_y : 1;
  uint32_t clip_accel_z : 1;
  uint32_t clip_gyro_x : 1;
  uint32_t clip_gyro_y : 1;
  uint32_t clip_gyro_z : 1;
  uint32_t clip_mag_x : 1;
  uint32_t clip_mag_y : 1;
  uint32_t clip_mag_z : 1;
  uint32_t reserved2 : 2;
  uint32_t clipping : 1;
  uint32_t reserved3 : 1;
  uint32_t sync_in : 1;
  uint32_t sync_out : 1;
  uint32_t filter_mode_raw : 3;
  uint32_t gnss_time_pulse : 1;
  uint32_t rtk_status_raw : 2;
  uint32_t reserved4 : 3;
};

static_assert(sizeof(StatusWord) == 4);

template <typename Derived, TypeId type_id, typename Precision, typename Coords>
struct Vector3 {
  static constexpr uint16_t id = GetDataId(type_id, Precision::id, Coords::id);
  static constexpr unsigned int size = 3 * Precision::size;

  static std::optional<Derived> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != size) return std::nullopt;

    unsigned int index = 0;

    Derived data;
    index += UnpackPrimitive<Precision>(data.x, buf + index);
    index += UnpackPrimitive<Precision>(data.y, buf + index);
    index += UnpackPrimitive<Precision>(data.z, buf + index);

    return data;
  }

  typename Precision::type x;
  typename Precision::type y;
  typename Precision::type z;
};

template <typename Derived, TypeId type_id, typename Precision, typename Coords>
struct QuaternionBase {
  static constexpr uint16_t id = GetDataId(type_id, Precision::id, Coords::id);
  static constexpr unsigned int size = 4 * Precision::size;

  static std::optional<Derived> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != size) return std::nullopt;

    unsigned int index = 0;

    Derived data;
    index += UnpackPrimitive<Precision>(data.w, buf + index);
    index += UnpackPrimitive<Precision>(data.x, buf + index);
    index += UnpackPrimitive<Precision>(data.y, buf + index);
    index += UnpackPrimitive<Precision>(data.z, buf + index);

    return data;
  }

  typename Precision::type w;
  typename Precision::type x;
  typename Precision::type y;
  typename Precision::type z;
};

struct UtcFlags {
  static unsigned int Unpack(UtcFlags& flags, const uint8_t *buf) {
    const uint8_t raw = buf[0];

    unsigned int offset = 0;
    flags.date_valid = util::UnpackField(raw, offset, 1);
    flags.day_valid = util::UnpackField(raw, offset, 1);
    flags.fully_resolved = util::UnpackField(raw, offset, 1);

    return 1;
  }

  uint8_t date_valid : 1;
  uint8_t day_valid : 1;
  uint8_t fully_resolved : 1;
};

// Struct specific types.
struct UtcTime {
  static constexpr uint16_t id = static_cast<uint16_t>(TypeId::kUtcTime);

  static std::optional<UtcTime> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != 12) return std::nullopt;

    unsigned int index = 0;

    UtcTime data;
    index += UnpackPrimitive<U4>(data.ns, buf + index);
    index += UnpackPrimitive<U2>(data.year, buf + index);
    index += UnpackPrimitive<U1>(data.month, buf + index);
    index += UnpackPrimitive<U1>(data.day, buf + index);
    index += UnpackPrimitive<U1>(data.hour, buf + index);
    index += UnpackPrimitive<U1>(data.minute, buf + index);
    index += UnpackPrimitive<U1>(data.second, buf + index);
    index += UtcFlags::Unpack(data.flags, buf + index);

    return data;
  }

  uint32_t ns;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  UtcFlags flags;
};

template <typename Precision, typename Coords>
struct Quaternion
    : QuaternionBase<Quaternion<Precision, Coords>, TypeId::kQuaternion, Precision, Coords> {};

template <typename Precision, typename Coords>
struct EulerAngles
    : Vector3<EulerAngles<Precision, Coords>, TypeId::kEulerAngles, Precision, Coords> {};

template <typename Precision, typename Coords>
struct RotationMatrix {
  static constexpr uint16_t id = GetDataId(TypeId::kRotationMatrix, Precision::id, Coords::id);

  static std::optional<RotationMatrix<Precision, Coords>> Unpack(const uint8_t *buf,
                                                                 unsigned int len) {
    if (len != 9 * Precision::size) return std::nullopt;

    unsigned int index = 0;

    RotationMatrix<Precision, Coords> data;
    index += UnpackPrimitive<Precision>(data.a, buf + index);
    index += UnpackPrimitive<Precision>(data.b, buf + index);
    index += UnpackPrimitive<Precision>(data.c, buf + index);
    index += UnpackPrimitive<Precision>(data.d, buf + index);
    index += UnpackPrimitive<Precision>(data.e, buf + index);
    index += UnpackPrimitive<Precision>(data.f, buf + index);
    index += UnpackPrimitive<Precision>(data.g, buf + index);
    index += UnpackPrimitive<Precision>(data.h, buf + index);
    index += UnpackPrimitive<Precision>(data.i, buf + index);

    return data;
  }

  typename Precision::type a;
  typename Precision::type b;
  typename Precision::type c;
  typename Precision::type d;
  typename Precision::type e;
  typename Precision::type f;
  typename Precision::type g;
  typename Precision::type h;
  typename Precision::type i;
};

template <typename Precision>
struct DeltaV : Vector3<DeltaV<Precision>, TypeId::kDeltaV, Precision, Sensor> {};

template <typename Precision>
struct DeltaQ : QuaternionBase<DeltaQ<Precision>, TypeId::kDeltaQ, Precision, Sensor> {};

template <typename Precision>
struct Acceleration : Vector3<Acceleration<Precision>, TypeId::kAcceleration, Precision, Sensor> {};

template <typename Precision, typename Coords>
struct FreeAcceleration
    : Vector3<FreeAcceleration<Precision, Coords>, TypeId::kFreeAcceleration, Precision, Coords> {};

template <typename Precision>
struct AccelerationHr
    : Vector3<AccelerationHr<Precision>, TypeId::kAccelerationHr, Precision, Sensor> {};

template <typename Precision>
struct RateOfTurn : Vector3<RateOfTurn<Precision>, TypeId::kRateOfTurn, Precision, Sensor> {};

template <typename Precision>
struct RateOfTurnHr : Vector3<RateOfTurnHr<Precision>, TypeId::kRateOfTurnHr, Precision, Sensor> {};

struct GnssPvtData {
  static constexpr uint16_t id = static_cast<uint16_t>(TypeId::kGnssPvtData);

  enum class FixType {
    kNone = 0x00,
    kDeadReckoning = 0x01,
    k2D = 0x02,
    k3D = 0x03,
    kDeadReckoningGnss = 0x04,
    kTime = 0x05,
  };

  static unsigned int UnpackFixType(FixType& fix_type, const uint8_t *buf) {
    fix_type = static_cast<FixType>(buf[0]);
    return 1;
  }

  struct FixStatusFlags {
    RtkSolution rtk_solution() const { return static_cast<RtkSolution>(rtk_solution_raw); }

    static unsigned int Unpack(FixStatusFlags& flags, const uint8_t *buf) {
      const uint8_t raw = buf[0];

      unsigned int offset = 0;
      flags.valid_fix = util::UnpackField(raw, offset, 1);
      flags.diff_corrections = util::UnpackField(raw, offset, 1);
      flags.reserved1 = util::UnpackField(raw, offset, 3);
      flags.valid_heading = util::UnpackField(raw, offset, 1);
      flags.rtk_solution_raw = util::UnpackField(raw, offset, 2);

      return 1;
    }

    uint8_t valid_fix : 1;
    uint8_t diff_corrections : 1;
    uint8_t reserved1 : 3;
    uint8_t valid_heading : 1;
    uint8_t rtk_solution_raw : 2;
  };

  static_assert(sizeof(FixStatusFlags) == 1);

  static std::optional<GnssPvtData> Unpack(const uint8_t *buf, unsigned int len) {
    if (len != 94) return std::nullopt;

    unsigned int index = 0;

    GnssPvtData data;
    index += UnpackPrimitive<U4>(data.itow, buf + index);
    index += UnpackPrimitive<U2>(data.year, buf + index);
    index += UnpackPrimitive<U1>(data.month, buf + index);
    index += UnpackPrimitive<U1>(data.day, buf + index);
    index += UnpackPrimitive<U1>(data.hour, buf + index);
    index += UnpackPrimitive<U1>(data.min, buf + index);
    index += UnpackPrimitive<U1>(data.sec, buf + index);
    index += UtcFlags::Unpack(data.valid, buf + index);
    index += UnpackPrimitive<U4>(data.t_acc, buf + index);
    index += UnpackPrimitive<I4>(data.nano, buf + index);
    index += UnpackFixType(data.fix_type, buf + index);
    index += FixStatusFlags::Unpack(data.flags, buf + index);
    index += UnpackPrimitive<U1>(data.num_sv, buf + index);
    index += UnpackPrimitive<U1>(data.reserved1, buf + index);
    index += UnpackPrimitive<I4>(data.lon, buf + index);
    index += UnpackPrimitive<I4>(data.lat, buf + index);
    index += UnpackPrimitive<I4>(data.height, buf + index);
    index += UnpackPrimitive<I4>(data.h_msl, buf + index);
    index += UnpackPrimitive<U4>(data.h_acc, buf + index);
    index += UnpackPrimitive<U4>(data.v_acc, buf + index);
    index += UnpackPrimitive<I4>(data.vel_n, buf + index);
    index += UnpackPrimitive<I4>(data.vel_e, buf + index);
    index += UnpackPrimitive<I4>(data.vel_d, buf + index);
    index += UnpackPrimitive<I4>(data.g_speed, buf + index);
    index += UnpackPrimitive<I4>(data.head_mot, buf + index);
    index += UnpackPrimitive<U4>(data.s_acc, buf + index);
    index += UnpackPrimitive<U4>(data.head_acc, buf + index);
    index += UnpackPrimitive<I4>(data.head_veh, buf + index);
    index += UnpackPrimitive<U2>(data.gdop, buf + index);
    index += UnpackPrimitive<U2>(data.pdop, buf + index);
    index += UnpackPrimitive<U2>(data.tdop, buf + index);
    index += UnpackPrimitive<U2>(data.vdop, buf + index);
    index += UnpackPrimitive<U2>(data.hdop, buf + index);
    index += UnpackPrimitive<U2>(data.ndop, buf + index);
    index += UnpackPrimitive<U2>(data.edop, buf + index);

    return data;
  }

  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  UtcFlags valid;
  uint32_t t_acc;
  int32_t nano;
  FixType fix_type;
  FixStatusFlags flags;
  uint8_t num_sv;
  uint8_t reserved1;
  int32_t lon;
  int32_t lat;
  int32_t height;
  int32_t h_msl;
  uint32_t h_acc;
  uint32_t v_acc;
  int32_t vel_n;
  int32_t vel_e;
  int32_t vel_d;
  int32_t g_speed;
  int32_t head_mot;
  uint32_t s_acc;
  uint32_t head_acc;
  int32_t head_veh;
  uint16_t gdop;
  uint16_t pdop;
  uint16_t tdop;
  uint16_t vdop;
  uint16_t hdop;
  uint16_t ndop;
  uint16_t edop;
};

struct GnssSatInfo {
  static constexpr uint16_t id = static_cast<uint16_t>(TypeId::kGnssSatInfo);

  struct GnssSatFlags {
    enum class SignalQuality {
      kNone = 0x00,
      kSearching = 0x01,
      kAquired = 0x02,
      kUnusable = 0x03,
      kCodeLocked = 0x04,
      kCarrierLocked = 0x05,
    };

    enum class Health {
      kUnknown = 0x00,
      kHealthy = 0x01,
      kUnhealthy = 0x02,
    };

    SignalQuality signal_quality() const {
      if (signal_quality_raw >= static_cast<uint8_t>(SignalQuality::kCarrierLocked)) {
        return SignalQuality::kCarrierLocked;
      }
      return static_cast<SignalQuality>(signal_quality_raw);
    }

    Health health() const { return static_cast<Health>(health_raw); }

    static unsigned int Unpack(GnssSatFlags& flags, const uint8_t *buf) {
      const uint8_t raw = buf[0];

      unsigned int offset = 0;
      flags.signal_quality_raw = util::UnpackField(raw, offset, 3);
      flags.used = util::UnpackField(raw, offset, 1);
      flags.health_raw = util::UnpackField(raw, offset, 2);
      flags.diff_corr_avail = util::UnpackField(raw, offset, 1);
      flags.reserved1 = util::UnpackField(raw, offset, 1);

      return 1;
    }

    uint8_t signal_quality_raw : 3;
    uint8_t used : 1;
    uint8_t health_raw : 2;
    uint8_t diff_corr_avail : 1;
    uint8_t reserved1 : 1;
  };

  enum class GnssId {
    kGps = 0x00,
    kSbas = 0x01,
    kGalileo = 0x02,
    kBeiDou = 0x03,
    kImes = 0x04,
    kQzss = 0x05,
    kGlonass = 0x06,
  };

  static unsigned int UnpackGnssId(GnssId& id, const uint8_t *buf) {
    id = static_cast<GnssId>(buf[0]);
    return 1;
  }

  struct GnssSatData {
    GnssId gnss_id;
    uint8_t sv_id;
    uint8_t cno;
    GnssSatFlags flags;
  };

  static std::optional<GnssSatInfo> Unpack(const uint8_t *buf, unsigned int len) {
    if (len > 8 + 64 * 4 || (len - 8) % 4) return std::nullopt;

    unsigned int index = 0;

    GnssSatInfo data;
    index += UnpackPrimitive<U4>(data.itow, buf + index);
    index += UnpackPrimitive<U1>(data.num_svs, buf + index);
    index += UnpackPrimitive<U1>(data.reserved1, buf + index);
    index += UnpackPrimitive<U1>(data.reserved2, buf + index);
    index += UnpackPrimitive<U1>(data.reserved3, buf + index);

    if (data.num_svs > 64) return std::nullopt;

    for (unsigned int i = 0; i < data.num_svs; ++i) {
      index += UnpackGnssId(data.sat_data[i].gnss_id, buf + index);
      index += UnpackPrimitive<U1>(data.sat_data[i].sv_id, buf + index);
      index += UnpackPrimitive<U1>(data.sat_data[i].cno, buf + index);
      index += GnssSatFlags::Unpack(data.sat_data[i].flags, buf + index);
    }

    return data;
  }

  uint32_t itow;
  uint8_t num_svs;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  GnssSatData sat_data[64];
};

struct RawAccGyrMagTemp {
  static constexpr uint16_t id = static_cast<uint16_t>(TypeId::kRawAccGyrMagTemp);

  struct Vector3U2 {
    uint16_t x;
    uint16_t y;
    uint16_t z;
  };

  static std::optional<RawAccGyrMagTemp> Unpack(const uint8_t *buf, unsigned int len) {
    if (len > 3 * 3 * 2 + 2) return std::nullopt;

    unsigned int index = 0;

    RawAccGyrMagTemp data;
    index += UnpackPrimitive<U2>(data.accel.x, buf + index);
    index += UnpackPrimitive<U2>(data.accel.y, buf + index);
    index += UnpackPrimitive<U2>(data.accel.z, buf + index);
    index += UnpackPrimitive<U2>(data.gyro.x, buf + index);
    index += UnpackPrimitive<U2>(data.gyro.y, buf + index);
    index += UnpackPrimitive<U2>(data.gyro.z, buf + index);
    index += UnpackPrimitive<U2>(data.mag.x, buf + index);
    index += UnpackPrimitive<U2>(data.mag.y, buf + index);
    index += UnpackPrimitive<U2>(data.mag.z, buf + index);
    index += UnpackPrimitive<I2>(data.temp, buf + index);

    return data;
  }

  Vector3U2 accel;
  Vector3U2 gyro;
  Vector3U2 mag;
  int16_t temp;
};

struct RawGyroTemp : Vector3<RawGyroTemp, TypeId::kRawGyroTemp, I2, Sensor> {};

template <typename Precision>
struct MagneticField
    : Vector3<MagneticField<Precision>, TypeId::kMagneticField, Precision, Sensor> {};

template <typename Precision>
struct PositionEcef : Vector3<PositionEcef<Precision>, TypeId::kPositionEcef, Precision, Sensor> {};

template <typename Precision>
struct LatLon {
  static constexpr uint16_t id =
      GetDataId(TypeId::kLatLon, Precision::id, CoordinateSystemId::kSensor);

  static std::optional<LatLon> Unpack(const uint8_t *buf, unsigned int len) {
    if (len > 2 * Precision::size) return std::nullopt;

    unsigned int index = 0;

    LatLon data;
    index += UnpackPrimitive<Precision>(data.lat, buf + index);
    index += UnpackPrimitive<Precision>(data.lon, buf + index);

    return data;
  }

  typename Precision::type lat;
  typename Precision::type lon;
};

template <typename Precision, typename Coords>
struct Velocity : Vector3<Velocity<Precision, Coords>, TypeId::kVelocity, Precision, Coords> {};

struct DataPacket {
  const uint8_t *data;
  uint8_t len;
};

static inline std::optional<DataPacket> FindDataPacket(uint16_t id, const uint8_t *buf,
                                                       unsigned int len) {
  unsigned int index = 0;
  while (index < len - 2) {
    const uint16_t packet_id = UnpackBigEndian16<uint16_t>(&buf[index + 0]);
    const uint8_t packet_len = buf[index + 2];
    const uint8_t *const packet_data = &buf[index + 3];

    if (packet_id == id) {
      // Buffer ends before end of data packet.
      if (packet_len + index + 3 > len) return std::nullopt;

      return DataPacket{.data = packet_data, .len = packet_len};
    }
    index += 3 + packet_len;
  }
  return std::nullopt;
}

template <typename T>
std::optional<DataPacket> GetDataPacket(const uint8_t *buf, unsigned int len) {
  return FindDataPacket(T::id, buf, len);
}

template <typename T>
std::optional<T> GetData(const uint8_t *buf, unsigned int len) {
  const std::optional<DataPacket> pkt = GetDataPacket<T>(buf, len);
  if (!pkt) return std::nullopt;
  return T::Unpack(pkt->data, pkt->len);
}

};  // namespace data
};  // namespace xsens
