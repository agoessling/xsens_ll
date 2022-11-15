#include <cstdint>

#include <gtest/gtest.h>

#include "msg_id.h"
#include "xbus_parser.h"

using namespace xbus;

TEST(Parser, ZeroLength) {
  uint8_t buf[0];
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, MsgError::kLen);
}

TEST(Parser, InsufficientLength) {
  uint8_t buf[4] = {0x00, 0x01, 0x02, 0x03};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, MsgError::kLen);
}

TEST(Parser, NoPreamble) {
  uint8_t buf[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, MsgError::kPreamble);
}

TEST(Parser, WrongBid) {
  uint8_t buf[5] = {0xFA, 0x01, 0x02, 0x03, 0x04};
  ParsedMsg msg = ParseMsg(buf, sizeof(buf));

  EXPECT_EQ(msg.error, MsgError::kBid);
}

TEST(Parser, WrongLength) {
  uint8_t buf[256] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0x2A};

  ParsedMsg msg = ParseMsg(buf, 46);
  EXPECT_EQ(msg.error, MsgError::kLen);

  msg = ParseMsg(buf, 48);
  EXPECT_NE(msg.error, MsgError::kNone);
}

TEST(Parser, ExtendedLengthWrongLength) {
  uint8_t buf[512] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0xFF, 0x01, 0xAA};

  ParsedMsg msg = ParseMsg(buf, 6);
  EXPECT_EQ(msg.error, MsgError::kLen);

  msg = ParseMsg(buf, 432);
  EXPECT_EQ(msg.error, MsgError::kLen);

  msg = ParseMsg(buf, 434);
  EXPECT_NE(msg.error, MsgError::kNone);
}

TEST(Parser, WrongChecksum) {
  uint8_t buf[512] = {0xFA, 0xFF, static_cast<uint8_t>(MsgId::kDeviceID), 0xFF, 0x01, 0xAA};

  ParsedMsg msg = ParseMsg(buf, 433);
  EXPECT_EQ(msg.error, MsgError::kChecksum);
}

TEST(Parser, Success) {
  uint8_t buf[] = {0xFA, 0xFF, 0xC0, 0x28, 0x10, 0x20, 0xFF, 0xFF, 0x10, 0x60, 0xFF, 0xFF,
                   0x20, 0x10, 0x00, 0x64, 0x40, 0x20, 0x01, 0x90, 0x80, 0x20, 0x01, 0x90,
                   0xC0, 0x20, 0x00, 0x64, 0xE0, 0x20, 0xFF, 0xFF, 0x50, 0x42, 0x00, 0x64,
                   0x50, 0x22, 0x00, 0x64, 0xD0, 0x12, 0x00, 0x64, 0x73};

  ParsedMsg msg = ParseMsg(buf, sizeof(buf));
  EXPECT_EQ(msg.error, MsgError::kNone);
  EXPECT_EQ(msg.id, MsgId::kSetOutputConfiguration);
  EXPECT_EQ(msg.len, 40);
  EXPECT_EQ(msg.data, buf + 4);
}
