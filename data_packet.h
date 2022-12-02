#pragma once

#include <optional>

#include "xbus_parser.h"

namespace xsens {
namespace data {

enum class Type {
  kTemperature = 0x0810  // [°C] Temperature
  kUtcTime = 0x1010  // [] UTC Time
  kPacketCounter = 0x1020  // [] Packet Counter
  kSampleTimeFine = 0x1060  // [] Sample Time Fine
  kSampleTimeCoarse = 0x1070  // [s] Sample Time Coarse
  kQuaternion = 0x2010  // [] Quaternion
  kRotationMatrix = 0x2020  // [] Rotation Matrix
  kEulerAngles = 0x2030  // [deg] Euler Angles
  kBaroPressure = 0x3010  // [Pa] Baro Pressure
  kDeltaV = 0x4010  // [m/s] Delta V
  kAcceleration = 0x4020  // [m/s2] Acceleration
  kFreeAcceleration = 0x4030  // [m/s2] Free Acceleration
  kAccelerationHR = 0x4040  // [m/s2] AccelerationHR
  kAltitudeEllipsoid = 0x5020  // [m] Altitude Ellipsoid
  kPositionEcef = 0x5030  // [m] Position ECEF
  kLatLon = 0x5040  // [deg] LatLon
  kGnssPvtData = 0x7010  // [] GNSS PVT data
  kGnssSatInfo = 0x7020  // [] GNSS satellites info
  kGnssPvtPulse = 0x7030  // [s] GNSS PVT pulse
  kRateOfTurn = 0x8020  // [rad/s] Rate of Turn
  kDeltaQ = 0x8030  // [] Delta Q
  kRateOfTurnHR = 0x8040  // [rad/s] RateOfTurnHR
  kRawAccGyrMagTemp = 0xA010  // [] ACC, GYR, MAG, temperature
  kRawGyroTemp = 0xA020  // [°C] Gyro temperatures
  kMagneticField = 0xC020  // [a.u.] Magnetic Field
  kVelocityXYZ = 0xD010  // [m/s] Velocity XYZ
  kStatusByte = 0xE010  // [] Status Byte
  kStatusWord = 0xE020  // [] Status Word
  kDeviceId = 0xE080  // [] Device ID
  kLocationId = 0xE090  // [] Location ID
};

enum class Precision {
  kFloat32 = 0x00,
  kFp1220 = 0x01,
  kFp1632 = 0x02,
  kFloat64 = 0x03,
};

enum class CoordinateSystem {
  kEnu = 0x00,
  kNed = 0x04,
  kNwu = 0x08,
};

struct Output {
  Type type;
  Precision precision;
  CoordinateSystem coordinates;
  uint16_t rate;
};

struct Float32 {
  using type = float;
  static constexpr Precision id = Precision::kFloat32;
};

struct Fp1220 {
  using type = uint32_t;
  static constexpr Precision id = Precision::kFp1220;
};

struct Fp1632 {
  using type = uint64_t;
  static constexpr Precision id = Precision::kFp1632;
};

struct Float64 {
  using type = double;
  static constexpr Precision id = Precision::kFloat64;
};

static constexpr uint16_t GetDataId(Type data_id, Precision precision_id,
                                    CoordinateSystem coords_id = kEnu) {
  return (static_cast<uint16_t>(data_id) | static_cast<uint16_t>(precision_id) |
          static_cast<uint16_t>(coords_id));
}

// Scalar Data Types.
using Temperature = struct {};
using PacketCounter = struct { uint16_t val; };
using SampleTimeFine = struct { uint32_t val; };
using SampleTimeCoarse = struct { uint32_t val; };
using BaroPressure = struct { uint32_t val; };
using GnssPvtPulse = struct { uint32_t val; };
using StatusByte = struct { uint8_t val; };
using StatusWord = struct { uint32_t val; };
using DeviceId = struct {};
using LocationId = struct { uint16_t val; };
using AltitudeEllipsoid = struct {};

// Struct Data Types.
struct UtcTime {
  uint32_t ns;
  uint16_t year;
  uint8_t month;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t flags;
};

template <typename Precision, CoordinateSystem coords>
struct Quaternion {
  static constexpr uint16_t id = GetDataId(Type::kQuaternion, Precision::id, coords);
  typename Precision::type w;
  typename Precision::type x;
  typename Precision::type y;
  typename Precision::type z;
};
//
// struct RealEulerAngles {
//  T roll;
//  T pitch;
//  T yaw;
//};
//
// struct RealRotationMatrix {
//  T a;
//  T b;
//  T c;
//  T d;
//  T e;
//  T f;
//  T g;
//  T h;
//  T i;
//};
//
// struct RealDeltaV {
//  T x;
//  T y;
//  T z;
//};
//
// struct RealDeltaQ {
//  T w;
//  T x;
//  T y;
//  T z;
//};
//
// struct RealAcceleration {
//  T x;
//  T y;
//  T z;
//};
//
// struct RealFreeAcceleration {
//  T x;
//  T y;
//  T z;
//};
//
// struct RealAccelerationHr {
//  T x;
//  T y;
//  T z;
//};
//
// struct RealRateOfTurn {
//  T x;
//  T y;
//  T z;
//};
//
// struct RealRateOfTurnHr {
//  T x;
//  T y;
//  T z;
//};

struct GnssPvtData {
  uint32_t itow;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t t_acc;
  int32_t nano;
  uint8_t fix_type;
  uint8_t flags;
  uint8_t num_sv;
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

struct GnssSatData {
  uint8_t gnss_id;
  uint8_t sv_id;
  uint8_t cno;
  uint8_t flags;
};

struct GnssSatInfo {
  uint32_t itow;
  uint8_t num_svs;
  GnssSateData sat_data[64]
};

struct RawAccGyrMagTemp {};
struct RawGyroTemp {};
struct MagneticField {};
struct PositionEcef {};
struct LatLon {};
struct VelocityXYZ {};

struct DataPacket {
  const uint8_t *data;
  uint8_t len;
};

std::optional<DataPacket> FindDataPacket(uint16_t id, const uint8_t *buf, unsigned int len) {
  unsigned int index = 0;
  while (index < len - 2) {
    const uint16_t packet_id = UnpackBigEndian16<uint16_t>(&buf[index + 0]);
    const uint8_t packet_len = buf[index + 2];
    const uint8_t *const packet_data = &buf[index + 3];

    if (packet_id == id) {
      // Buffer ends before end of data packet.
      if (packet_len + index + 3 > len) return std::nullopt;

      return {.data = packet_data, .len = packet_len};
    }
    index += 3 + packet_len;
  }
  return std::nullopt;
}

};  // namespace data
};  // namespace xsens
