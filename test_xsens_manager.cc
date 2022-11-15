#include <cstdint>
#include <deque>
#include <vector>

#include <gtest/gtest.h>

#include "xsens_manager.h"

using namespace xbus;

namespace read_fake {

static bool g_error = false;
static std::deque<uint8_t> g_data;
static int g_read_calls = 0;

static inline int Read(uint8_t *buf, unsigned int len) {
  ++g_read_calls;

  if (g_error) {
    return -1;
  }

  for (unsigned int i = 0; i < len; ++i) {
    if (g_data.empty()) {
      return i;
    }

    buf[i] = g_data.front();
    g_data.pop_front();
  }

  return len;
}

static inline void Reset() {
  g_error = false;
  g_data.clear();
  g_read_calls = 0;
}

static inline int GetReadCalls() {
  return g_read_calls;
}

static inline void SetError(bool error) {
  g_error = error;
}

static inline void AddData(const std::vector<uint8_t>& data) {
  g_data.insert(g_data.end(), data.begin(), data.end());
}

};  // namespace read_fake

TEST(XsensManager, ReadError) {
  XsensManager manager(read_fake::Read);

  read_fake::Reset();

  read_fake::SetError(true);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorReadCall);
}

TEST(XsensManager, NoMsg) {
  XsensManager manager(read_fake::Read);

  // No data.
  read_fake::Reset();

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReading);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);

  // Some data.
  read_fake::Reset();

  read_fake::AddData({0x01, 0x02, 0x03, 0x04});

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReading);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);

  // Multiple reads.
  read_fake::Reset();

  read_fake::AddData(std::vector<uint8_t>(65, 0xAA));

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReading);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
}

TEST(XsensManager, SingleMsg) {
  const std::vector<uint8_t> set_baudrate_msg = {0xFA, 0xFF, 0x18, 0x01, 0x80, 0x68};
  const std::vector<uint8_t> go_to_config_msg = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

  XsensManager manager(read_fake::Read);

  // Single message.
  read_fake::Reset();

  read_fake::AddData(set_baudrate_msg);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  // Single message with random start.
  read_fake::Reset();

  read_fake::AddData({0xAA, 0xAA, 0xAA});
  read_fake::AddData(set_baudrate_msg);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  // Single message with long random start.
  read_fake::Reset();

  read_fake::AddData(std::vector<uint8_t>(63, 0xAA));
  read_fake::AddData(set_baudrate_msg);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);
}

TEST(XsensManager, MultipleMsgs) {
  const std::vector<uint8_t> set_baudrate_msg = {0xFA, 0xFF, 0x18, 0x01, 0x80, 0x68};
  const std::vector<uint8_t> go_to_config_msg = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

  XsensManager manager(read_fake::Read);

  // Multiple messages.
  read_fake::Reset();

  read_fake::AddData(set_baudrate_msg);
  read_fake::AddData(go_to_config_msg);
  read_fake::AddData(set_baudrate_msg);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], go_to_config_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  // Multiple messages with random start.
  read_fake::Reset();

  read_fake::AddData({0xAA, 0xAA, 0xAA});
  read_fake::AddData(set_baudrate_msg);
  read_fake::AddData(go_to_config_msg);
  read_fake::AddData(set_baudrate_msg);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], go_to_config_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  // Multiple messages with long random start.
  read_fake::Reset();

  read_fake::AddData(std::vector<uint8_t>(63, 0xAA));
  read_fake::AddData(set_baudrate_msg);
  read_fake::AddData(go_to_config_msg);
  read_fake::AddData(set_baudrate_msg);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], go_to_config_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 4);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);
}

TEST(XsensManager, MsgsWithErrors) {
  const std::vector<uint8_t> set_baudrate_msg = {0xFA, 0xFF, 0x18, 0x01, 0x80, 0x68};
  const std::vector<uint8_t> go_to_config_msg = {0xFA, 0xFF, 0x30, 0x00, 0xD1};

  XsensManager manager(read_fake::Read);

  // Messages preceeded with errors.
  read_fake::Reset();

  read_fake::AddData({kPreambleVal, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA});
  read_fake::AddData(set_baudrate_msg);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorParse);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.error, ParseError::kBid);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  // Multiple messages interspersed with errors.
  read_fake::Reset();

  read_fake::AddData(set_baudrate_msg);
  read_fake::AddData({kPreambleVal, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA});
  read_fake::AddData(go_to_config_msg);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorParse);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.error, ParseError::kBid);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], go_to_config_msg[4]);
}

TEST(XsensManager, LongMsg) {
  XsensManager manager(read_fake::Read);

  read_fake::Reset();

  std::vector<uint8_t> buf(400);
  buf[0] = kPreambleVal;
  buf[1] = 0xFF;  // BID
  buf[2] = 0x00;  // MID
  buf[3] = 0xFF;  // LEN
  buf[4] = 0x01;  // EXT LEN HIGH
  buf[5] = 0x89;  // EXT LEN LOW
  buf[6] = 0xAA;  // DATA
  buf[399] = 0xCE;  // CHECKSUM

  read_fake::AddData(buf);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 7);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 393);
  EXPECT_EQ(info.msg.data[0], buf[6]);
}

TEST(XsensManager, PreambleChecksum) {
  std::vector<uint8_t> buf = {kPreambleVal, 0xFF, 0x00, 0x01, 0x06, kPreambleVal};
  const std::vector<uint8_t> set_baudrate_msg = {0xFA, 0xFF, 0x18, 0x01, 0x80, 0x68};

  XsensManager manager(read_fake::Read);

  read_fake::Reset();

  read_fake::AddData(buf);
  read_fake::AddData(set_baudrate_msg);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], buf[4]);

  info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(read_fake::GetReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], set_baudrate_msg[4]);
}

TEST(XsensManager, MsgTooLarge) {
  XsensManager manager(read_fake::Read);

  read_fake::Reset();

  // Really long extended length message.
  std::vector<uint8_t> buf(1024);
  buf[0] = kPreambleVal;
  buf[1] = 0xFF;  // BID
  buf[2] = 0x01;  // MID
  buf[3] = 0xFF;  // LEN
  buf[4] = 0xFF;  // EXT LEN HIGH
  buf[5] = 0xFF;  // EXT LEN LOW

  read_fake::AddData(buf);

  XsensManager::MsgInfo info = manager.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorMsgTooLarge);
  EXPECT_EQ(read_fake::GetReadCalls(), 1);
}
