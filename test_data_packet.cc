#include <cstdint>
#include <numeric>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "bitfield.h"
#include "data_packet.h"

using namespace xsens::data;
using namespace xsens::util;

const static uint8_t kByte1Val = 0xA0;
const static std::vector<uint8_t> kByte1Buf{0xA0};

const static uint16_t kByte2Val = 0xAF24;
const static std::vector<uint8_t> kByte2Buf{0xAF, 0x24};

const static uint32_t kByte4Val = 0xA7BEC186;
const static std::vector<uint8_t> kByte4Buf{0xA7, 0xBE, 0xC1, 0x86};

const static uint64_t kByte6Val = 0xFFFFFFA3B4C8DA57;
const static std::vector<uint8_t> kByte6Buf{0xFF, 0xA3, 0xB4, 0xC8, 0xDA, 0x57};

const static uint64_t kByte8Val = 0x708192A1B7C2D617;
const static std::vector<uint8_t> kByte8Buf{0x70, 0x81, 0x92, 0xA1, 0xB7, 0xC2, 0xD6, 0x17};

template <typename T>
void Extend(std::vector<T>& a, const std::vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}

static inline std::vector<uint8_t> Range(unsigned int start, unsigned int stop) {
  std::vector<uint8_t> data(stop - start);
  std::iota(data.begin(), data.end(), start);
  return data;
}

static inline std::vector<uint8_t> Add(const std::vector<uint8_t>& buf, uint8_t a) {
  std::vector<uint8_t> data = buf;
  data.back() += a;
  return data;
}

template <typename T, typename U>
T Pun(U data) {
  static_assert(sizeof(T) == sizeof(U));
  return reinterpret_cast<T&>(data);
}

TEST(GetData, Temperature) {
  std::vector<uint8_t> buf{0x08, 0x10, 0x04};
  Extend(buf, kByte4Buf);

  auto data = GetData<Temperature<Float32>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, Pun<float>(kByte4Val));
}

TEST(GetData, UtcTime) {
  std::vector<uint8_t> buf{0x10, 0x10, 0x0C};
  Extend(buf, Range(1, 13));

  auto data = GetData<UtcTime>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->ns, 0x01020304);
  EXPECT_EQ(data->year, 0x0506);
  EXPECT_EQ(data->month, 0x07);
  EXPECT_EQ(data->day, 0x08);
  EXPECT_EQ(data->hour, 0x09);
  EXPECT_EQ(data->minute, 0x0A);
  EXPECT_EQ(data->second, 0x0B);
  EXPECT_EQ(data->flags.date_valid, GetField(0x0CU, 0, 1));
  EXPECT_EQ(data->flags.day_valid, GetField(0x0CU, 1, 1));
  EXPECT_EQ(data->flags.fully_resolved, GetField(0x0CU, 2, 1));
}

TEST(GetData, PacketCounter) {
  std::vector<uint8_t> buf{0x10, 0x20, 0x02};
  Extend(buf, {0x01, 0x02});

  auto data = GetData<PacketCounter>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x0102);
}

TEST(GetData, SampleTimeFine) {
  std::vector<uint8_t> buf{0x10, 0x60, 0x04};
  Extend(buf, {0x01, 0x02, 0x03, 0x04});

  auto data = GetData<SampleTimeFine>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x01020304);
}

TEST(GetData, SampleTimeCoarse) {
  std::vector<uint8_t> buf{0x10, 0x70, 0x04};
  Extend(buf, {0x01, 0x02, 0x03, 0x04});

  auto data = GetData<SampleTimeCoarse>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x01020304);
}

TEST(GetData, Quaternion) {
  std::vector<uint8_t> buf{0x20, 0x1A, 0x18};
  Extend(buf, Add(kByte6Buf, 0));
  Extend(buf, Add(kByte6Buf, 1));
  Extend(buf, Add(kByte6Buf, 2));
  Extend(buf, Add(kByte6Buf, 3));

  auto data = GetData<Quaternion<Fp1632, Nwu>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->w, Pun<int64_t>(kByte6Val + 0));
  EXPECT_EQ(data->x, Pun<int64_t>(kByte6Val + 1));
  EXPECT_EQ(data->y, Pun<int64_t>(kByte6Val + 2));
  EXPECT_EQ(data->z, Pun<int64_t>(kByte6Val + 3));
}

TEST(GetData, EulerAngles) {
  std::vector<uint8_t> buf{0x20, 0x33, 0x18};
  Extend(buf, Add(kByte8Buf, 0));
  Extend(buf, Add(kByte8Buf, 1));
  Extend(buf, Add(kByte8Buf, 2));

  auto data = GetData<EulerAngles<Float64, Enu>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<double>(kByte8Val + 0));
  EXPECT_EQ(data->y, Pun<double>(kByte8Val + 1));
  EXPECT_EQ(data->z, Pun<double>(kByte8Val + 2));
}

TEST(GetData, RotationMatrix) {
  std::vector<uint8_t> buf{0x20, 0x25, 0x24};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));
  Extend(buf, Add(kByte4Buf, 3));
  Extend(buf, Add(kByte4Buf, 4));
  Extend(buf, Add(kByte4Buf, 5));
  Extend(buf, Add(kByte4Buf, 6));
  Extend(buf, Add(kByte4Buf, 7));
  Extend(buf, Add(kByte4Buf, 8));

  auto data = GetData<RotationMatrix<Fp1220, Ned>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->a, Pun<int32_t>(kByte4Val + 0));
  EXPECT_EQ(data->b, Pun<int32_t>(kByte4Val + 1));
  EXPECT_EQ(data->c, Pun<int32_t>(kByte4Val + 2));
  EXPECT_EQ(data->d, Pun<int32_t>(kByte4Val + 3));
  EXPECT_EQ(data->e, Pun<int32_t>(kByte4Val + 4));
  EXPECT_EQ(data->f, Pun<int32_t>(kByte4Val + 5));
  EXPECT_EQ(data->g, Pun<int32_t>(kByte4Val + 6));
  EXPECT_EQ(data->h, Pun<int32_t>(kByte4Val + 7));
  EXPECT_EQ(data->i, Pun<int32_t>(kByte4Val + 8));
}

TEST(GetData, BaroPressure) {
  std::vector<uint8_t> buf{0x30, 0x10, 0x04};
  Extend(buf, {0x01, 0x02, 0x03, 0x04});

  auto data = GetData<BaroPressure>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x01020304);
}

TEST(GetData, DeltaV) {
  std::vector<uint8_t> buf{0x40, 0x10, 0x0C};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));

  auto data = GetData<DeltaV<Float32>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<float>(kByte4Val + 0));
  EXPECT_EQ(data->y, Pun<float>(kByte4Val + 1));
  EXPECT_EQ(data->z, Pun<float>(kByte4Val + 2));
}

TEST(GetData, DeltaQ) {
  std::vector<uint8_t> buf{0x80, 0x32, 0x18};
  Extend(buf, Add(kByte6Buf, 0));
  Extend(buf, Add(kByte6Buf, 1));
  Extend(buf, Add(kByte6Buf, 2));
  Extend(buf, Add(kByte6Buf, 3));

  auto data = GetData<DeltaQ<Fp1632>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->w, Pun<int64_t>(kByte6Val + 0));
  EXPECT_EQ(data->x, Pun<int64_t>(kByte6Val + 1));
  EXPECT_EQ(data->y, Pun<int64_t>(kByte6Val + 2));
  EXPECT_EQ(data->z, Pun<int64_t>(kByte6Val + 3));
}

TEST(GetData, Acceleration) {
  std::vector<uint8_t> buf{0x40, 0x20, 0x0C};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));

  auto data = GetData<Acceleration<Float32>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<float>(kByte4Val + 0));
  EXPECT_EQ(data->y, Pun<float>(kByte4Val + 1));
  EXPECT_EQ(data->z, Pun<float>(kByte4Val + 2));
}

TEST(GetData, FreeAcceleration) {
  std::vector<uint8_t> buf{0x40, 0x37, 0x18};
  Extend(buf, Add(kByte8Buf, 0));
  Extend(buf, Add(kByte8Buf, 1));
  Extend(buf, Add(kByte8Buf, 2));

  auto data = GetData<FreeAcceleration<Float64, Ned>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<double>(kByte8Val + 0));
  EXPECT_EQ(data->y, Pun<double>(kByte8Val + 1));
  EXPECT_EQ(data->z, Pun<double>(kByte8Val + 2));
}

TEST(GetData, AccelerationHr) {
  std::vector<uint8_t> buf{0x40, 0x40, 0x0C};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));

  auto data = GetData<AccelerationHr<Float32>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<float>(kByte4Val + 0));
  EXPECT_EQ(data->y, Pun<float>(kByte4Val + 1));
  EXPECT_EQ(data->z, Pun<float>(kByte4Val + 2));
}

TEST(GetData, RateOfTurn) {
  std::vector<uint8_t> buf{0x80, 0x21, 0x0C};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));

  auto data = GetData<RateOfTurn<Fp1220>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<int32_t>(kByte4Val + 0));
  EXPECT_EQ(data->y, Pun<int32_t>(kByte4Val + 1));
  EXPECT_EQ(data->z, Pun<int32_t>(kByte4Val + 2));
}

TEST(GetData, RateOfTurnHr) {
  std::vector<uint8_t> buf{0x80, 0x42, 0x12};
  Extend(buf, Add(kByte6Buf, 0));
  Extend(buf, Add(kByte6Buf, 1));
  Extend(buf, Add(kByte6Buf, 2));

  auto data = GetData<RateOfTurnHr<Fp1632>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<int64_t>(kByte6Val + 0));
  EXPECT_EQ(data->y, Pun<int64_t>(kByte6Val + 1));
  EXPECT_EQ(data->z, Pun<int64_t>(kByte6Val + 2));
}

TEST(GetData, GnssPvtData) {
  std::vector<uint8_t> buf{0x70, 0x10, 0x5E};
  Extend(buf, Range(1, 95));

  auto data = GetData<GnssPvtData>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->itow, 0x01020304);
  EXPECT_EQ(data->year, 0x0506);
  EXPECT_EQ(data->month, 0x07);
  EXPECT_EQ(data->day, 0x08);
  EXPECT_EQ(data->hour, 0x09);
  EXPECT_EQ(data->min, 0x0A);
  EXPECT_EQ(data->sec, 0x0B);
  EXPECT_EQ(data->valid.date_valid, GetField(0x0C, 0, 1));
  EXPECT_EQ(data->valid.day_valid, GetField(0x0C, 1, 1));
  EXPECT_EQ(data->valid.fully_resolved, GetField(0x0C, 2, 1));
  EXPECT_EQ(data->t_acc, 0x0D0E0F10);
  EXPECT_EQ(data->nano, 0x11121314);
  EXPECT_EQ(data->fix_type, static_cast<GnssPvtData::FixType>(0x15));
  EXPECT_EQ(data->flags.valid_fix, GetField(0x16, 0, 1));
  EXPECT_EQ(data->flags.diff_corrections, GetField(0x16, 1, 1));
  EXPECT_EQ(data->flags.reserved1, GetField(0x16, 2, 3));
  EXPECT_EQ(data->flags.valid_heading, GetField(0x16, 5, 1));
  EXPECT_EQ(data->flags.rtk_solution_raw, GetField(0x16, 6, 2));
  EXPECT_EQ(data->num_sv, 0x17);
  EXPECT_EQ(data->reserved1, 0x18);
  EXPECT_EQ(data->lon, 0x191A1B1C);
  EXPECT_EQ(data->lat, 0x1D1E1F20);
  EXPECT_EQ(data->height, 0x21222324);
  EXPECT_EQ(data->h_msl, 0x25262728);
  EXPECT_EQ(data->h_acc, 0x292A2B2C);
  EXPECT_EQ(data->v_acc, 0x2D2E2F30);
  EXPECT_EQ(data->vel_n, 0x31323334);
  EXPECT_EQ(data->vel_e, 0x35363738);
  EXPECT_EQ(data->vel_d, 0x393A3B3C);
  EXPECT_EQ(data->g_speed, 0x3D3E3F40);
  EXPECT_EQ(data->head_mot, 0x41424344);
  EXPECT_EQ(data->s_acc, 0x45464748);
  EXPECT_EQ(data->head_acc, 0x494A4B4C);
  EXPECT_EQ(data->head_veh, 0x4D4E4F50);
  EXPECT_EQ(data->gdop, 0x5152);
  EXPECT_EQ(data->pdop, 0x5354);
  EXPECT_EQ(data->tdop, 0x5556);
  EXPECT_EQ(data->vdop, 0x5758);
  EXPECT_EQ(data->hdop, 0x595A);
  EXPECT_EQ(data->ndop, 0x5B5C);
  EXPECT_EQ(data->edop, 0x5D5E);
}

TEST(GetData, GnssSatInfo) {
  std::vector<uint8_t> buf{0x70, 0x20, 8 + 3 * 4};
  Extend(buf, {0x01, 0x02, 0x03, 0x04, 0x03, 0x05, 0x06, 0x07});
  Extend(buf, Range(1, 3 * 4 + 1));

  auto data = GetData<GnssSatInfo>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->itow, 0x01020304);
  EXPECT_EQ(data->num_svs, 0x03);
  EXPECT_EQ(data->reserved1, 0x05);
  EXPECT_EQ(data->reserved2, 0x06);
  EXPECT_EQ(data->reserved3, 0x07);
  EXPECT_EQ(data->sat_data[0].gnss_id, static_cast<GnssSatInfo::GnssId>(0x01));
  EXPECT_EQ(data->sat_data[0].sv_id, 0x02);
  EXPECT_EQ(data->sat_data[0].cno, 0x03);
  EXPECT_EQ(data->sat_data[0].flags.signal_quality_raw, GetField(0x04, 0, 3));
  EXPECT_EQ(data->sat_data[0].flags.used, GetField(0x04, 3, 1));
  EXPECT_EQ(data->sat_data[0].flags.health_raw, GetField(0x04, 4, 2));
  EXPECT_EQ(data->sat_data[0].flags.diff_corr_avail, GetField(0x04, 6, 1));
  EXPECT_EQ(data->sat_data[0].flags.reserved1, GetField(0x04, 7, 1));
  EXPECT_EQ(data->sat_data[1].gnss_id, static_cast<GnssSatInfo::GnssId>(0x05));
  EXPECT_EQ(data->sat_data[1].sv_id, 0x06);
  EXPECT_EQ(data->sat_data[1].cno, 0x07);
  EXPECT_EQ(data->sat_data[1].flags.signal_quality_raw, GetField(0x08, 0, 3));
  EXPECT_EQ(data->sat_data[1].flags.used, GetField(0x08, 3, 1));
  EXPECT_EQ(data->sat_data[1].flags.health_raw, GetField(0x08, 4, 2));
  EXPECT_EQ(data->sat_data[1].flags.diff_corr_avail, GetField(0x08, 6, 1));
  EXPECT_EQ(data->sat_data[1].flags.reserved1, GetField(0x08, 7, 1));
  EXPECT_EQ(data->sat_data[2].gnss_id, static_cast<GnssSatInfo::GnssId>(0x09));
  EXPECT_EQ(data->sat_data[2].sv_id, 0x0A);
  EXPECT_EQ(data->sat_data[2].cno, 0x0B);
  EXPECT_EQ(data->sat_data[2].flags.signal_quality_raw, GetField(0x0C, 0, 3));
  EXPECT_EQ(data->sat_data[2].flags.used, GetField(0x0C, 3, 1));
  EXPECT_EQ(data->sat_data[2].flags.health_raw, GetField(0x0C, 4, 2));
  EXPECT_EQ(data->sat_data[2].flags.diff_corr_avail, GetField(0x0C, 6, 1));
  EXPECT_EQ(data->sat_data[2].flags.reserved1, GetField(0x0C, 7, 1));
}

TEST(GetData, GnssPvtPulse) {
  std::vector<uint8_t> buf{0x70, 0x30, 0x04};
  Extend(buf, {0x01, 0x02, 0x03, 0x04});

  auto data = GetData<GnssPvtPulse>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x01020304);
}

TEST(GetData, RawAccGyrMagTemp) {
  std::vector<uint8_t> buf{0xA0, 0x10, 3 * 3 * 2 + 2};
  Extend(buf, Range(1, 3 * 3 * 2 + 3));

  auto data = GetData<RawAccGyrMagTemp>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->accel.x, 0x0102);
  EXPECT_EQ(data->accel.y, 0x0304);
  EXPECT_EQ(data->accel.z, 0x0506);
  EXPECT_EQ(data->gyro.x, 0x0708);
  EXPECT_EQ(data->gyro.y, 0x090A);
  EXPECT_EQ(data->gyro.z, 0x0B0C);
  EXPECT_EQ(data->mag.x, 0x0D0E);
  EXPECT_EQ(data->mag.y, 0x0F10);
  EXPECT_EQ(data->mag.z, 0x1112);
  EXPECT_EQ(data->temp, 0x1314);
}

TEST(GetData, RawGyroTemp) {
  std::vector<uint8_t> buf{0xA0, 0x20, 6};
  Extend(buf, Range(1, 7));

  auto data = GetData<RawGyroTemp>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, 0x0102);
  EXPECT_EQ(data->y, 0x0304);
  EXPECT_EQ(data->z, 0x0506);
}

TEST(GetData, MagneticField) {
  std::vector<uint8_t> buf{0xC0, 0x23, 0x18};
  Extend(buf, Add(kByte8Buf, 0));
  Extend(buf, Add(kByte8Buf, 1));
  Extend(buf, Add(kByte8Buf, 2));

  auto data = GetData<MagneticField<Float64>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<double>(kByte8Val + 0));
  EXPECT_EQ(data->y, Pun<double>(kByte8Val + 1));
  EXPECT_EQ(data->z, Pun<double>(kByte8Val + 2));
}

TEST(GetData, StatusByte) {
  std::vector<uint8_t> buf{0xE0, 0x10, 0x01};
  Extend(buf, {0xAA});

  auto data = GetData<StatusByte>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->self_test, GetField(0xAA, 0, 1));
  EXPECT_EQ(data->filter_valid, GetField(0xAA, 1, 1));
  EXPECT_EQ(data->gnss_fix, GetField(0xAA, 2, 1));
  EXPECT_EQ(data->no_rotation_status_raw, GetField(0xAA, 3, 2));
  EXPECT_EQ(data->rep_mo, GetField(0xAA, 5, 1));
  EXPECT_EQ(data->clock_sync, GetField(0xAA, 6, 1));
  EXPECT_EQ(data->reserved1, GetField(0xAA, 7, 1));
}

TEST(GetData, StatusWord) {
  std::vector<uint8_t> buf{0xE0, 0x20, 0x04};
  Extend(buf, {0xBA, 0x98, 0x76, 0x54});

  auto data = GetData<StatusWord>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->status_byte.self_test, GetField(0xBA987654, 0, 1));
  EXPECT_EQ(data->status_byte.filter_valid, GetField(0xBA987654, 1, 1));
  EXPECT_EQ(data->status_byte.gnss_fix, GetField(0xBA987654, 2, 1));
  EXPECT_EQ(data->status_byte.no_rotation_status_raw, GetField(0xBA987654, 3, 2));
  EXPECT_EQ(data->status_byte.rep_mo, GetField(0xBA987654, 5, 1));
  EXPECT_EQ(data->status_byte.clock_sync, GetField(0xBA987654, 6, 1));
  EXPECT_EQ(data->status_byte.reserved1, GetField(0xBA987654, 7, 1));
  EXPECT_EQ(data->clip_accel_x, GetField(0xBA987654, 8, 1));
  EXPECT_EQ(data->clip_accel_y, GetField(0xBA987654, 9, 1));
  EXPECT_EQ(data->clip_accel_z, GetField(0xBA987654, 10, 1));
  EXPECT_EQ(data->clip_gyro_x, GetField(0xBA987654, 11, 1));
  EXPECT_EQ(data->clip_gyro_y, GetField(0xBA987654, 12, 1));
  EXPECT_EQ(data->clip_gyro_z, GetField(0xBA987654, 13, 1));
  EXPECT_EQ(data->clip_mag_x, GetField(0xBA987654, 14, 1));
  EXPECT_EQ(data->clip_mag_y, GetField(0xBA987654, 15, 1));
  EXPECT_EQ(data->clip_mag_z, GetField(0xBA987654, 16, 1));
  EXPECT_EQ(data->reserved2, GetField(0xBA987654, 17, 2));
  EXPECT_EQ(data->clipping, GetField(0xBA987654, 19, 1));
  EXPECT_EQ(data->reserved3, GetField(0xBA987654, 20, 1));
  EXPECT_EQ(data->sync_in, GetField(0xBA987654, 21, 1));
  EXPECT_EQ(data->sync_out, GetField(0xBA987654, 22, 1));
  EXPECT_EQ(data->filter_mode_raw, GetField(0xBA987654, 23, 3));
  EXPECT_EQ(data->gnss_time_pulse, GetField(0xBA987654, 26, 1));
  EXPECT_EQ(data->rtk_status_raw, GetField(0xBA987654, 27, 2));
  EXPECT_EQ(data->reserved4, GetField(0xBA987654, 29, 3));
}

TEST(GetData, DeviceIdU4) {
  std::vector<uint8_t> buf{0xE0, 0x80, 0x04};
  Extend(buf, Range(1, 5));

  auto data = GetData<DeviceIdU4>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x01020304);
}

TEST(GetData, DeviceIdU8) {
  std::vector<uint8_t> buf{0xE0, 0x80, 0x08};
  Extend(buf, Range(1, 9));

  auto data = GetData<DeviceIdU8>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x0102030405060708);
}

TEST(GetData, LocationId) {
  std::vector<uint8_t> buf{0xE0, 0x90, 0x02};
  Extend(buf, Range(1, 3));

  auto data = GetData<LocationId>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, 0x0102);
}

TEST(GetData, PositionEcef) {
  std::vector<uint8_t> buf{0x50, 0x30, 0x0C};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));
  Extend(buf, Add(kByte4Buf, 2));

  auto data = GetData<PositionEcef<Float32>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<float>(kByte4Val + 0));
  EXPECT_EQ(data->y, Pun<float>(kByte4Val + 1));
  EXPECT_EQ(data->z, Pun<float>(kByte4Val + 2));
}

TEST(GetData, LatLon) {
  std::vector<uint8_t> buf{0x50, 0x41, 0x08};
  Extend(buf, Add(kByte4Buf, 0));
  Extend(buf, Add(kByte4Buf, 1));

  auto data = GetData<LatLon<Fp1220>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->lat, Pun<int32_t>(kByte4Val + 0));
  EXPECT_EQ(data->lon, Pun<int32_t>(kByte4Val + 1));
}

TEST(GetData, AltitudeEllipsoid) {
  std::vector<uint8_t> buf{0x50, 0x22, 0x06};
  Extend(buf, kByte6Buf);

  auto data = GetData<AltitudeEllipsoid<Fp1632>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->val, Pun<int64_t>(kByte6Val));
}

TEST(GetData, Velocity) {
  std::vector<uint8_t> buf{0xD0, 0x1B, 0x18};
  Extend(buf, Add(kByte8Buf, 0));
  Extend(buf, Add(kByte8Buf, 1));
  Extend(buf, Add(kByte8Buf, 2));

  auto data = GetData<Velocity<Float64, Nwu>>(buf.data(), buf.size());
  ASSERT_TRUE(data);
  EXPECT_EQ(data->x, Pun<double>(kByte8Val + 0));
  EXPECT_EQ(data->y, Pun<double>(kByte8Val + 1));
  EXPECT_EQ(data->z, Pun<double>(kByte8Val + 2));
}

TEST(DataId, GetTypeId) {
  EXPECT_EQ(GetTypeId(0x202F), TypeId::kRotationMatrix);
  EXPECT_EQ(GetTypeId(0x702F), TypeId::kGnssSatInfo);
}

TEST(DataId, GetPrecisionId) {
  EXPECT_EQ(GetPrecisionId(0xFFFC), PrecisionId::kFloat32);
  EXPECT_EQ(GetPrecisionId(0xFFFD), PrecisionId::kFp1220);
  EXPECT_EQ(GetPrecisionId(0xFFFE), PrecisionId::kFp1632);
  EXPECT_EQ(GetPrecisionId(0xFFFF), PrecisionId::kFloat64);
}

TEST(DataId, GetCoordinateSystemId) {
  EXPECT_EQ(GetCoordinateSystemId(0xFFF3), CoordinateSystemId::kEnu);
  EXPECT_EQ(GetCoordinateSystemId(0xFFF7), CoordinateSystemId::kNed);
  EXPECT_EQ(GetCoordinateSystemId(0xFFFB), CoordinateSystemId::kNwu);
}

TEST(FindDataPacket, Present) {
  std::vector<uint8_t> buf{0xAA, 0xAA, 0x01, 0xAA, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

  std::optional<DataPacket> pkt = FindDataPacket(0x0102, buf.data(), buf.size());
  ASSERT_TRUE(pkt);
  EXPECT_EQ(pkt->data, buf.data() + 7);
  EXPECT_EQ(pkt->len, 3);
}

TEST(FindDataPacket, Missing) {
  std::vector<uint8_t> buf{0xAA, 0xAA, 0x01, 0xAA, 0x02, 0x02, 0x03, 0x04, 0x05, 0x06};

  std::optional<DataPacket> pkt = FindDataPacket(0x0102, buf.data(), buf.size());
  EXPECT_FALSE(pkt);
}

TEST(FindDataPacket, WrongLength) {
  std::vector<uint8_t> buf{0xAA, 0xAA, 0x01, 0xAA, 0x02, 0x02, 0x03, 0x04, 0x05};

  std::optional<DataPacket> pkt = FindDataPacket(0x0102, buf.data(), buf.size());
  EXPECT_FALSE(pkt);
}

TEST(FindDataPacket, Malformed) {
  std::vector<uint8_t> buf{0xAA, 0xAA, 0x04, 0x01, 0x02, 0x02, 0x03, 0x04, 0x05};

  std::optional<DataPacket> pkt = FindDataPacket(0x0102, buf.data(), buf.size());
  EXPECT_FALSE(pkt);
}
