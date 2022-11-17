#include <cstdint>

#include <gtest/gtest.h>

#include "msg_id.h"
#include "xbus_parser.h"

using namespace xbus;

TEST(Pack, BigEndian16) {
  const int16_t original = -3284;

  uint8_t buf[2];
  PackBigEndian16(original, buf);

  EXPECT_EQ(buf[0], 0xF3);
  EXPECT_EQ(buf[1], 0x2C);

  const int16_t unpacked = UnpackBigEndian16<int16_t>(buf);

  EXPECT_EQ(unpacked, original);
}

TEST(Pack, BigEndian32) {
  const int32_t original = -1234567;

  uint8_t buf[4];
  PackBigEndian32(original, buf);

  EXPECT_EQ(buf[0], 0xFF);
  EXPECT_EQ(buf[1], 0xED);
  EXPECT_EQ(buf[2], 0x29);
  EXPECT_EQ(buf[3], 0x79);

  const int32_t unpacked = UnpackBigEndian32<int32_t>(buf);

  EXPECT_EQ(unpacked, original);
}

TEST(Pack, BigEndian64) {
  const int64_t original = -12345678901;

  uint8_t buf[8];
  PackBigEndian64(original, buf);

  EXPECT_EQ(buf[0], 0xFF);
  EXPECT_EQ(buf[1], 0xFF);
  EXPECT_EQ(buf[2], 0xFF);
  EXPECT_EQ(buf[3], 0xFD);
  EXPECT_EQ(buf[4], 0x20);
  EXPECT_EQ(buf[5], 0x23);
  EXPECT_EQ(buf[6], 0xE3);
  EXPECT_EQ(buf[7], 0xCB);

  const int64_t unpacked = UnpackBigEndian64<int64_t>(buf);

  EXPECT_EQ(unpacked, original);
}

TEST(Parser, ZeroLength) {
  uint8_t buf[0];
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, ParseError::kLen);
}

TEST(Parser, InsufficientLength) {
  uint8_t buf[4] = {0x00, 0x01, 0x02, 0x03};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, ParseError::kLen);
}

TEST(Parser, NoPreamble) {
  uint8_t buf[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, ParseError::kPreamble);
}

TEST(Parser, WrongBid) {
  uint8_t buf[5] = {0xFA, 0x01, 0x02, 0x03, 0x04};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, ParseError::kBid);
}

TEST(Parser, WrongLength) {
  uint8_t buf[256] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0x2A};

  ParsedMsg msg = ParseMsg(buf, 46);
  EXPECT_EQ(msg.error, ParseError::kLen);

  msg = ParseMsg(buf, 48);
  EXPECT_NE(msg.error, ParseError::kNone);
}

TEST(Parser, ExtendedLengthWrongLength) {
  uint8_t buf[512] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0xFF, 0x01, 0xAA};

  ParsedMsg msg = ParseMsg(buf, 6);
  EXPECT_EQ(msg.error, ParseError::kLen);

  msg = ParseMsg(buf, 432);
  EXPECT_EQ(msg.error, ParseError::kLen);

  msg = ParseMsg(buf, 434);
  EXPECT_NE(msg.error, ParseError::kNone);
}

TEST(Parser, WrongChecksum) {
  uint8_t buf[512] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0xFF, 0x01, 0xAA};

  ParsedMsg msg = ParseMsg(buf, 433);
  EXPECT_EQ(msg.error, ParseError::kChecksum);
}

TEST(Parser, Success) {
  // Set output configuration message with random byte on end.
  uint8_t buf[] = {0xFA, 0xFF, 0xC0, 0x28, 0x10, 0x20, 0xFF, 0xFF, 0x10, 0x60, 0xFF, 0xFF,
                   0x20, 0x10, 0x00, 0x64, 0x40, 0x20, 0x01, 0x90, 0x80, 0x20, 0x01, 0x90,
                   0xC0, 0x20, 0x00, 0x64, 0xE0, 0x20, 0xFF, 0xFF, 0x50, 0x42, 0x00, 0x64,
                   0x50, 0x22, 0x00, 0x64, 0xD0, 0x12, 0x00, 0x64, 0x73, 0xAA};

  ParsedMsg msg = ParseMsg(buf, sizeof(buf));
  EXPECT_EQ(msg.error, ParseError::kNone);
  EXPECT_EQ(msg.id, MsgId::kSetOutputConfiguration);
  EXPECT_EQ(msg.len, 40);
  EXPECT_EQ(msg.data, buf + 4);
}

TEST(Parser, ExtendedLengthSuccess) {
  // Set output configuration message with random byte on end.
  uint8_t buf[519] = {0xFA, 0xFF, 0xC0, 0xFF, 0x02, 0x00};
  buf[518] = 0x40;

  ParsedMsg msg = ParseMsg(buf, sizeof(buf));
  EXPECT_EQ(msg.error, ParseError::kNone);
  EXPECT_EQ(msg.id, MsgId::kSetOutputConfiguration);
  EXPECT_EQ(msg.len, 512);
  EXPECT_EQ(msg.data, buf + 6);
}

TEST(PackMsg, BufferTooSmall) {
  uint8_t buf[512] = {};
  uint8_t data[512] = {};

  EXPECT_EQ(PackMsg(buf, 10, MsgId::kDeviceID, data, 5).error, PackError::kNone);
  EXPECT_EQ(PackMsg(buf, 10, MsgId::kDeviceID, data, 6).error, PackError::kOverflow);

  EXPECT_EQ(PackMsg(buf, 300, MsgId::kDeviceID, data, 293).error, PackError::kNone);
  EXPECT_EQ(PackMsg(buf, 300, MsgId::kDeviceID, data, 294).error, PackError::kOverflow);
}

TEST(PackMsg, DataTooLarge) {
  uint8_t *buf = new uint8_t[UINT16_MAX + 10]{};
  uint8_t *data = new uint8_t[UINT16_MAX + 10]{};

  EXPECT_EQ(PackMsg(buf, UINT16_MAX + 10, MsgId::kDeviceID, data, UINT16_MAX).error,
            PackError::kNone);
  EXPECT_EQ(PackMsg(buf, UINT16_MAX + 10, MsgId::kDeviceID, data, UINT16_MAX + 1).error,
            PackError::kDataTooLarge);

  delete[] buf;
  delete[] data;
}

TEST(PackMsg, RegularMsg) {
  uint8_t buf[32];
  const uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
  const unsigned int packed_len = sizeof(data) + 5;

  PackResult result = PackMsg(buf, sizeof(buf), MsgId::kDeviceID, data, sizeof(data));

  EXPECT_EQ(result.error, PackError::kNone);
  EXPECT_EQ(result.len, packed_len);
  EXPECT_EQ(buf[0], kPreambleVal);
  EXPECT_EQ(buf[1], kBidVal);
  EXPECT_EQ(buf[2], static_cast<uint8_t>(MsgId::kDeviceID));
  EXPECT_EQ(buf[3], sizeof(data));

  for (unsigned int i = 0; i < sizeof(data); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(buf[4 + i], data[i]);
  }

  uint8_t checksum = 0;
  for (unsigned int i = 1; i < packed_len; ++i) {
    checksum += buf[i];
  }

  EXPECT_EQ(checksum, 0);
}

TEST(PackMsg, ExtendedMsg) {
  uint8_t buf[300];
  const uint8_t data[275] = {0x01, 0x02, 0x03, 0x04};
  const unsigned int packed_len = sizeof(data) + 7;

  PackResult result = PackMsg(buf, sizeof(buf), MsgId::kDeviceID, data, sizeof(data));

  EXPECT_EQ(result.error, PackError::kNone);
  EXPECT_EQ(result.len, packed_len);
  EXPECT_EQ(buf[0], kPreambleVal);
  EXPECT_EQ(buf[1], kBidVal);
  EXPECT_EQ(buf[2], static_cast<uint8_t>(MsgId::kDeviceID));
  EXPECT_EQ(buf[3], 0xFF);
  EXPECT_EQ(buf[4], static_cast<uint8_t>(sizeof(data) >> 8));
  EXPECT_EQ(buf[5], static_cast<uint8_t>(sizeof(data)));

  for (unsigned int i = 0; i < sizeof(data); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(buf[6 + i], data[i]);
  }

  uint8_t checksum = 0;
  for (unsigned int i = 1; i < packed_len; ++i) {
    checksum += buf[i];
  }

  EXPECT_EQ(checksum, 0);
}
