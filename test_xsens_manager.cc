#include <cstdint>
#include <deque>
#include <vector>

#include <gtest/gtest.h>

#include "xsens_manager.h"

using namespace xbus;

class FakeXsensManager : public XsensManager {
 public:
  FakeXsensManager() { Reset(); }

  void Reset() {
    epoch_time_us_ = 0;
    read_data_.clear();
    write_data_.clear();
    read_calls_ = 0;
    write_calls_ = 0;
    read_error_ = false;
    write_error_ = false;
  }

  void SetReadError(bool error) { read_error_ = error; }
  void SetWriteError(bool error) { write_error_ = error; }

  int ReadCalls() { return read_calls_; }
  int WriteCalls() { return write_calls_; }

  void AppendReadData(const std::vector<uint8_t>& data) {
    read_data_.insert(read_data_.end(), data.begin(), data.end());
  }

  std::vector<uint8_t>& WriteData() { return write_data_; }

 private:
  int ReadBytes(uint8_t *buf, unsigned int len) override final {
    ++read_calls_;

    // Simulate 10ms per read call.
    epoch_time_us_ += 10000;

    if (read_error_) {
      return -1;
    }

    for (unsigned int i = 0; i < len; ++i) {
      if (read_data_.empty()) {
        return i;
      }

      buf[i] = read_data_.front();
      read_data_.pop_front();
    }

    return len;
  }

  int WriteBytes(const uint8_t *buf, unsigned int len) override final {
    ++write_calls_;

    if (write_error_) {
      return -1;
    }

    for (unsigned int i = 0; i < len; ++i) {
      write_data_.push_back(buf[i]);
    }

    return len;
  }

  uint64_t EpochTimeUs() override final { return epoch_time_us_; }

  uint64_t epoch_time_us_;
  std::deque<uint8_t> read_data_;
  std::vector<uint8_t> write_data_;
  int read_calls_;
  int write_calls_;
  bool read_error_;
  bool write_error_;
};

class XsensManagerTest : public ::testing::Test {
 public:
  XsensManagerTest()
      : kSetBaudrateMsg{0xFA, 0xFF, 0x18, 0x01, 0x80, 0x68},
        kGoToConfigMsg{0xFA, 0xFF, 0x30, 0x00, 0xD1} {}

 protected:
  const std::vector<uint8_t> kSetBaudrateMsg;
  const std::vector<uint8_t> kGoToConfigMsg;

  FakeXsensManager manager_;
};

TEST_F(XsensManagerTest, ReadError) {
  manager_.SetReadError(true);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorReadCall);
}

TEST_F(XsensManagerTest, NoMsg) {
  // No data.
  manager_.Reset();

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kNoMsg);
  EXPECT_EQ(manager_.ReadCalls(), 1);

  // Some data.
  manager_.Reset();

  manager_.AppendReadData({0x01, 0x02, 0x03, 0x04});

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kNoMsg);
  EXPECT_EQ(manager_.ReadCalls(), 1);

  // Multiple reads.
  manager_.Reset();

  manager_.AppendReadData(std::vector<uint8_t>(65, 0xAA));

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kNoMsg);
  EXPECT_EQ(manager_.ReadCalls(), 2);
}

TEST_F(XsensManagerTest, SingleMsg) {
  // Single message.
  manager_.Reset();

  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  // Single message with random start.
  manager_.Reset();

  manager_.AppendReadData({0xAA, 0xAA, 0xAA});
  manager_.AppendReadData(kSetBaudrateMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  // Single message with long random start.
  manager_.Reset();

  manager_.AppendReadData(std::vector<uint8_t>(63, 0xAA));
  manager_.AppendReadData(kSetBaudrateMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);
}

TEST_F(XsensManagerTest, MultipleMsgs) {
  // Multiple messages.
  manager_.Reset();

  manager_.AppendReadData(kSetBaudrateMsg);
  manager_.AppendReadData(kGoToConfigMsg);
  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], kGoToConfigMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  // Multiple messages with random start.
  manager_.Reset();

  manager_.AppendReadData({0xAA, 0xAA, 0xAA});
  manager_.AppendReadData(kSetBaudrateMsg);
  manager_.AppendReadData(kGoToConfigMsg);
  manager_.AppendReadData(kSetBaudrateMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], kGoToConfigMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  // Multiple messages with long random start.
  manager_.Reset();

  manager_.AppendReadData(std::vector<uint8_t>(63, 0xAA));
  manager_.AppendReadData(kSetBaudrateMsg);
  manager_.AppendReadData(kGoToConfigMsg);
  manager_.AppendReadData(kSetBaudrateMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], kGoToConfigMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 4);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);
}

TEST_F(XsensManagerTest, MsgsWithErrors) {
  // Messages preceeded with errors.
  manager_.Reset();

  manager_.AppendReadData({kPreambleVal, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA});
  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorParse);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.error, ParseError::kBid);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  // Multiple messages interspersed with errors.
  manager_.Reset();

  manager_.AppendReadData(kSetBaudrateMsg);
  manager_.AppendReadData({kPreambleVal, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA});
  manager_.AppendReadData(kGoToConfigMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorParse);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.error, ParseError::kBid);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], kGoToConfigMsg[4]);
}

TEST_F(XsensManagerTest, LongMsg) {
  std::vector<uint8_t> buf(400);
  buf[0] = kPreambleVal;
  buf[1] = 0xFF;  // BID
  buf[2] = 0x00;  // MID
  buf[3] = 0xFF;  // LEN
  buf[4] = 0x01;  // EXT LEN HIGH
  buf[5] = 0x89;  // EXT LEN LOW
  buf[6] = 0xAA;  // DATA
  buf[399] = 0xCE;  // CHECKSUM

  manager_.AppendReadData(buf);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 7);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 393);
  EXPECT_EQ(info.msg.data[0], buf[6]);
}

TEST_F(XsensManagerTest, PreambleChecksum) {
  std::vector<uint8_t> buf = {kPreambleVal, 0xFF, 0x00, 0x01, 0x06, kPreambleVal};

  manager_.AppendReadData(buf);
  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], buf[4]);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);
}

TEST_F(XsensManagerTest, MsgTooLarge) {
  // Really long extended length message.
  std::vector<uint8_t> buf(1024);
  buf[0] = kPreambleVal;
  buf[1] = 0xFF;  // BID
  buf[2] = 0x01;  // MID
  buf[3] = 0xFF;  // LEN
  buf[4] = 0xFF;  // EXT LEN HIGH
  buf[5] = 0xFF;  // EXT LEN LOW

  manager_.AppendReadData(buf);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kErrorMsgTooLarge);
  EXPECT_EQ(manager_.ReadCalls(), 1);
}

TEST_F(XsensManagerTest, AdvanceReadBuffer) {
  std::vector<uint8_t> buf = {kPreambleVal, 0xFF, 0x00, 0xAA};  // Incomplete message.

  manager_.AppendReadData(buf);
  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReadingMsg);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 0xAA);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReadingMsg);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 0xAA);

  manager_.AdvanceReadBuffer();

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kSetBaudrate);
  EXPECT_EQ(info.msg.len, 1);
  EXPECT_EQ(info.msg.data[0], kSetBaudrateMsg[4]);
}

TEST_F(XsensManagerTest, ResetReadBuffer) {
  std::vector<uint8_t> buf = {kPreambleVal, 0xFF, 0x00, 0xAA};  // Incomplete message.
                                                                //
  manager_.AppendReadData(buf);
  manager_.AppendReadData(kSetBaudrateMsg);

  XsensManager::MsgInfo info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReadingMsg);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 0xAA);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kReadingMsg);
  EXPECT_EQ(manager_.ReadCalls(), 2);
  EXPECT_EQ(info.msg.id, MsgId::kReqDID);
  EXPECT_EQ(info.msg.len, 0xAA);

  manager_.ResetReadBuffer();

  manager_.AppendReadData(kGoToConfigMsg);

  info = manager_.ReadMsg();

  EXPECT_EQ(info.status, XsensManager::ReadStatus::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 3);
  EXPECT_EQ(info.msg.id, MsgId::kGoToConfig);
  EXPECT_EQ(info.msg.len, 0);
  EXPECT_EQ(info.msg.data[0], kGoToConfigMsg[4]);
}

TEST_F(XsensManagerTest, GoToConfig) {
  uint8_t msg[32];
  uint8_t ack[32];

  const unsigned int msg_len = PackMsg(msg, sizeof(msg), MsgId::kGoToConfig, nullptr, 0).len;
  const unsigned int ack_len = PackMsg(ack, sizeof(ack), MsgId::kGoToConfigAck, nullptr, 0).len;

  const std::vector<uint8_t> msg_vec(msg, msg + msg_len);
  const std::vector<uint8_t> ack_vec(ack, ack + ack_len);

  manager_.AppendReadData(ack_vec);
  XsensManager::ConfigResult result = manager_.GoToConfig();

  EXPECT_EQ(result, XsensManager::ConfigResult::kSuccess);
  EXPECT_EQ(manager_.ReadCalls(), 1);
  EXPECT_EQ(manager_.WriteCalls(), 1);
  EXPECT_EQ(manager_.WriteData(), msg_vec);
}

TEST_F(XsensManagerTest, GoToConfigTimeout) {
  uint8_t msg[32];
  uint8_t ack[32];

  const unsigned int msg_len = PackMsg(msg, sizeof(msg), MsgId::kGoToConfig, nullptr, 0).len;
  const unsigned int ack_len = PackMsg(ack, sizeof(ack), MsgId::kGoToConfigAck, nullptr, 0).len;

  const std::vector<uint8_t> msg_vec(msg, msg + msg_len);
  const std::vector<uint8_t> ack_vec(ack, ack + ack_len);

  XsensManager::ConfigResult result = manager_.GoToConfig();

  EXPECT_EQ(result, XsensManager::ConfigResult::kErrorTimeout);
  EXPECT_GT(manager_.ReadCalls(), 1);
  EXPECT_EQ(manager_.WriteCalls(), 1);
  EXPECT_EQ(manager_.WriteData(), msg_vec);
}
