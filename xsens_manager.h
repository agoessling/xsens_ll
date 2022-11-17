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
  };

  struct MsgInfo {
    ReadStatus status;
    ParsedMsg msg;
  };

  XsensManager() : write_index_{0}, start_index_{0} {}

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

  ConfigResult GoToConfig() {
    PackResult result = PackMsg(write_buf_, kWriteBufLen, MsgId::kGoToConfig, nullptr, 0);

    if (result.error != PackError::kNone) {
      return ConfigResult::kErrorPack;
    }

    const uint64_t now_us = EpochTimeUs();

    if (!WriteAllBytesBlocking(write_buf_, result.len)) {
      return ConfigResult::kErrorWriteCall;
    }

    while (EpochTimeUs() - now_us < kConfigTimeoutUs) {
      const MsgInfo info = ReadMsg();

      if (info.status == ReadStatus::kSuccess) {
        if (info.msg.id == MsgId::kGoToConfigAck) {
          return ConfigResult::kSuccess;
        }

        if (info.msg.id == MsgId::kError) {
          return ConfigResult::kErrorConfig;
        }

        continue;
      }

      if (info.status == ReadStatus::kNoMsg || info.status == ReadStatus::kReadingMsg) {
        continue;
      }

      return ConfigResult::kErrorRead;
    }

    return ConfigResult::kErrorTimeout;
  }

 private:
  static constexpr unsigned int kReadBufLen = 512;
  static constexpr unsigned int kWriteBufLen = 512;
  static constexpr unsigned int kBytesPerRead = 64;
  static constexpr unsigned int kConfigTimeoutUs = 100000;

  // Must be overridden by subclass.
  virtual int ReadBytes(uint8_t *buf, unsigned int len) = 0;
  virtual int WriteBytes(const uint8_t *buf, unsigned int len) = 0;
  virtual uint64_t EpochTimeUs() = 0;

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

  uint8_t read_buf_[kReadBufLen];
  uint8_t write_buf_[kWriteBufLen];
  unsigned int write_index_;
  unsigned int start_index_;
  int (*read_func_)(uint8_t *buf, unsigned int len);
};

};  // namespace xbus
