#pragma once

#include <cstdint>
#include <cstring>

#include "xbus_parser.h"

namespace xbus {

class XsensManager {
 public:
  enum class ReadStatus {
    kSuccess,  // Parsed message available.
    kNoMsg,  // No message received.
    kReadingMsg,  // Waiting for reception of rest of message.
    kErrorOverflow,  // Receive buffer overflow.
    kErrorMsgTooLarge,  // Received message too large.
    kErrorReadCall,  // Error during user provided read call.
    kErrorParse,  // Error with parsing.  See nested msg.error for further info.
  };

  enum class ConfigResult {
    kSuccess,
    kErrorTimeout,  // Timeout while waiting on acknowledgement
    kErrorPack,  // Error in packing.
    kErrorWriteCall,  // Error during call to WriteBytes.
    kErrorConfig,  // Xsens responded with an error.
    kErrorRead,  // Error during MsgRead.
    kErrorLen,  // Response length incorrect.
  };

  struct MsgInfo {
    ReadStatus status;
    ParsedMsg msg;
  };

  XsensManager(unsigned int timeout_us)
      : write_index_{0},
        start_index_{0},
        default_timeout_us_{timeout_us},
        timeout_us_{default_timeout_us_} {}

  MsgInfo ReadMsg() {
    MsgInfo info = {
        .status = ReadStatus::kNoMsg,
        .msg = {.len = 0, .data = nullptr},
    };

    bool data_available = true;
    while (data_available) {
      // Check for space in receive buffer. (Don't think this can ever be exercised)
      if (kReadBufLen - write_index_ < kBytesPerRead) {
        info.status = ReadStatus::kErrorOverflow;
        return info;
      }

      const int bytes_read = ReadBytes(read_buf_ + write_index_, kBytesPerRead);
      data_available = bytes_read == kBytesPerRead;

      if (bytes_read < 0) {
        info.status = ReadStatus::kErrorReadCall;
        return info;
      }

      write_index_ += bytes_read;

      // Shift rx buffer to preamble and continue if none found.
      const bool preamble_found = ShiftMsgBufferToPreamble(start_index_);
      start_index_ = 0;

      if (!preamble_found) {
        continue;
      }

      info.msg = ParseMsg(read_buf_, write_index_);

      if (info.msg.error == ParseError::kNone) {
        info.status = ReadStatus::kSuccess;

        // Next iteration start after message end.
        start_index_ = (info.msg.data - read_buf_) + info.msg.len + 1;

        return info;
      }

      // If error anything other than insufficient length.
      if (info.msg.error != ParseError::kLen) {
        info.status = ReadStatus::kErrorParse;

        // Next iteration start past this preamble.
        start_index_ = 1;

        return info;
      }

      // Message too big.
      if (info.msg.len + kMaxMsgOverhead > kReadBufLen) {
        info.status = ReadStatus::kErrorMsgTooLarge;

        // Next iteration start past this preamble.
        start_index_ = 1;

        return info;
      }

      // Currently reading message.
      info.status = ReadStatus::kReadingMsg;
    }

    return info;
  }

  void AdvanceReadBuffer() { ++start_index_; }

  void ResetReadBuffer() {
    write_index_ = 0;
    start_index_ = 0;
  }

  ConfigResult GoToConfig() { return SendConfig(MsgId::kGoToConfig).result; }

  ConfigResult GetDeviceId(uint32_t *device_id) {
    ConfigResponse resp = SendConfig(MsgId::kReqDID);

    if (resp.result != ConfigResult::kSuccess) return resp.result;

    if (resp.len == 4) {
      *device_id = UnpackBigEndian32<uint32_t>(resp.data);
      return ConfigResult::kSuccess;
    }
    if (resp.len == 8) {
      *device_id = UnpackBigEndian32<uint32_t>(resp.data + 4);
      return ConfigResult::kSuccess;
    }

    return ConfigResult::kErrorLen;
  }

  ConfigResult GetProductCode(const char **str, unsigned int *len) {
    ConfigResponse resp = SendConfig(MsgId::kReqProductCode);

    if (resp.result != ConfigResult::kSuccess) return resp.result;

    *str = reinterpret_cast<const char *>(resp.data);
    *len = resp.len;

    return ConfigResult::kSuccess;
  }

  ConfigResult RunSelfTest(bool *pass) {
    SetTimeoutUs(1'000'000);
    ConfigResponse resp = SendConfig(MsgId::kRunSelftest);
    RestoreTimeout();

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 2) return ConfigResult::kErrorLen;

    uint16_t test = UnpackBigEndian16<uint16_t>(resp.data);

    *pass = (test & 0x7FF) == 0x7FF;
    return ConfigResult::kSuccess;
  }

 private:
  static constexpr unsigned int kReadBufLen = 512;
  static constexpr unsigned int kWriteBufLen = 512;
  static constexpr unsigned int kBytesPerRead = 64;

  struct ConfigResponse {
    ConfigResult result;
    const uint8_t *data;
    unsigned int len;
  };

  // Must be overridden by subclass.
  virtual int ReadBytes(uint8_t *buf, unsigned int len) = 0;
  virtual int FlushBytes() = 0;
  virtual int WriteBytes(const uint8_t *buf, unsigned int len) = 0;
  virtual uint64_t EpochTimeUs() = 0;

  void SetTimeoutUs(unsigned int timeout_us) { timeout_us_ = timeout_us; }
  void RestoreTimeout() { timeout_us_ = default_timeout_us_; }

  int FindPreamble(unsigned int start_ind) {
    for (int i = start_ind; i < static_cast<int>(write_index_); ++i) {
      if (read_buf_[i] == kPreambleVal) {
        return i;
      }
    }

    return -1;
  }

  void ShiftMsgBuffer(unsigned int start_ind) {
    // If desired start index at or above write_index_ no need to actually shift.
    if (write_index_ <= start_ind) {
      write_index_ = 0;
      return;
    }

    memmove(read_buf_, read_buf_ + start_ind, write_index_ - start_ind);
    write_index_ -= start_ind;
  }

  // Returns whether preamble is found.
  bool ShiftMsgBufferToPreamble(unsigned int start_ind) {
    const int preamble_ind = FindPreamble(start_ind);

    if (preamble_ind < 0) {
      write_index_ = 0;
      return false;
    }

    if (preamble_ind > 0) {
      ShiftMsgBuffer(preamble_ind);
    }

    return true;
  }

  bool WriteAllBytesBlocking(const uint8_t *buf, unsigned int len) {
    unsigned int bytes_left = len;
    while (bytes_left > 0) {
      int bytes_written = WriteBytes(buf + len - bytes_left, bytes_left);

      if (bytes_written < 0) {
        return false;
      }

      bytes_left -= bytes_written;
    }

    return true;
  }

  ConfigResponse SendConfig(MsgId id) {
    ConfigResponse resp;

    PackResult result = PackMsg(write_buf_, kWriteBufLen, id, nullptr, 0);

    if (result.error != PackError::kNone) {
      resp.result = ConfigResult::kErrorPack;
      return resp;
    }

    if (!WriteAllBytesBlocking(write_buf_, result.len)) {
      resp.result = ConfigResult::kErrorWriteCall;
      return resp;
    }

    const uint64_t now_us = EpochTimeUs();
    while (EpochTimeUs() - now_us < timeout_us_) {
      const MsgInfo info = ReadMsg();

      if (info.status == ReadStatus::kSuccess) {
        // Most response IDs are incremented by one.
        if (info.msg.id == static_cast<MsgId>(static_cast<int>(id) + 1)) {
          resp.result = ConfigResult::kSuccess;
          resp.data = info.msg.data;
          resp.len = info.msg.len;
          return resp;
        }

        if (info.msg.id == MsgId::kError) {
          resp.result = ConfigResult::kErrorConfig;
          return resp;
        }
      }
    }

    resp.result = ConfigResult::kErrorTimeout;
    return resp;
  }

  uint8_t read_buf_[kReadBufLen];
  uint8_t write_buf_[kWriteBufLen];
  unsigned int write_index_;
  unsigned int start_index_;
  unsigned int default_timeout_us_;
  unsigned int timeout_us_;
};

};  // namespace xbus
