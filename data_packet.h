#pragma once

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

// Base types.
using Float32 = float;
using Fp1220 = uint32_t;
using Fp1632 = uint64_t;
using Float64 = double;
using U1 = uint8_t;
using U2 = uint16_t;
using U4 = uint32_t;
using I1 = int8_t;
using I2 = int16_t;
using I4 = int32_t;

using RealScalar = std::variant<Float32, Fp1220, Fp1632, Float64>;

template <template<typename> typename T>
using RealStruct = std::variant<T<Float32>, T<Fp1220>, T<Fp1632>, T<Float64>>;

// Scalar Data Types.
using Temperature = struct { RealScalar val; };
using PacketCounter = struct { U2 val; };
using SampleTimeFine = struct { U4 val; };
using SampleTimeCoarse = struct { U4 val; };
using BaroPressure = struct { U4 val; };
using GnssPvtPulse = struct { U4 val; };
using StatusByte = struct { U1 val; };
using StatusWord = struct { U4 val; };
using DeviceId = struct { std::variant<U4, U8> val; };
using LocationId = struct { U2 val; };
using AltitudeEllipsoid = struct { RealScalar val; };

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

template <typename T>
struct RealQuaternion {
  T w;
  T x;
  T y;
  T z;
};
using Quaternion = RealStruct<RealQuaternion>;

template <typename T>
struct RealEulerAngles {
  T roll;
  T pitch;
  T yaw;
};
using EulerAngles = RealStruct<RealEulerAngles>;

template <typename T>
struct RealRotationMatrix {
  T a;
  T b;
  T c;
  T d;
  T e;
  T f;
  T g;
  T h;
  T i;
};
using RotationMatrix = RealStruct<RealRotationMatrix>;

template <typename T>
struct RealDeltaV {
  T x;
  T y;
  T z;
};
using DeltaV = RealStruct<RealDeltaV>;

template <typename T>
struct RealDeltaQ {
  T w;
  T x;
  T y;
  T z;
};
using DeltaQ = RealStruct<RealDeltaQ>;

template <typename T>
struct RealAcceleration {
  T x;
  T y;
  T z;
};
using Acceleration = RealStruct<RealAcceleration>;

template <typename T>
struct RealFreeAcceleration {
  T x;
  T y;
  T z;
};
using FreeAcceleration = RealStruct<RealFreeAcceleration>;

template <typename T>
struct RealAccelerationHr {
  T x;
  T y;
  T z;
};
using AccelerationHr = RealStruct<RealAccelerationHr>;

template <typename T>
struct RealRateOfTurn {
  T x;
  T y;
  T z;
};
using RateOfTurn = RealStruct<RealRateOfTurn>;

template <typename T>
struct RealRateOfTurnHr {
  T x;
  T y;
  T z;
};
using RateOfTurnHr = RealStruct<RealRateOfTurnHr>;

struct GnssPvtData {
  U4 itow;
  U2 year;
  U1 month;
  U1 day;
  U1 hour;
  U1 min;
  U1 sec;
  U1 valid;
  U4 t_acc;
  I4 nano;
  U1 fix_type;
  U1 flags;
  U1 num_sv;
  I4 lon;
  I4 lat;
  I4 height;
  I4 h_msl;
  U4 h_acc;
  U4 v_acc;
  I4 vel_n;
  I4 vel_e;
  I4 vel_d;
  I4 g_speed;
  I4 head_mot;
  U4 s_acc;
  U4 head_acc;
  I4 head_veh;
  U2 gdop;
  U2 pdop;
  U2 tdop;
  U2 vdop;
  U2 hdop;
  U2 ndop;
  U2 edop;
};

struct GnssSatData {
  U1 gnss_id;
  U1 sv_id;
  U1 cno;
  U1 flags;
};

struct GnssSatInfo {
  U4 itow;
  U1 num_svs;
  GnssSateData sat_data[64]
};

struct RawAccGyrMagTemp
struct RawGyroTemp
struct MagneticField
struct PositionEcef
struct LatLon
struct VelocityXYZ


using DataType = std::variant<Temperature, AltitudeEllipsoid, SampleTimeCoarse, UtcTime, Quaternion>;

struct DataPacket {
  const uint8_t *data;
  uint8_t len;
};

static inline std::optional<DataPacket> FindDataPacket(uint16_t id, const uint8_t *buf, unsigned int len) {
  unsigned int index = 0;
  while (index < len - 2) {
    const uint16_t packet_id = UnpackBigEndian16<uint16_t>(&buf[index + 0]);
    const uint8_t packet_len = buf[index + 2];
    const uint8_t *const packet_data = &buf[index + 3];

    if (packet_id == id) {
      // Buffer ends before end of data packet.
      if (packet_len + index + 3 > len) return std::nullopt;

      return { .data = packet_data, .len = packet_len };
    }
    index += 3 + packet_len;
  }
  return std::nullopt;
}

};  // namespace data
};  // namespace xsens
