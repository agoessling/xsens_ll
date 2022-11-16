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
      if (kMsgBufLen - write_index_ < kBytesPerRead) {
        info.status = ReadStatus::kErrorOverflow;
        return info;
      }

      const int bytes_read = ReadBytes(msg_buf_ + write_index_, kBytesPerRead);
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

      info.msg = ParseMsg(msg_buf_, write_index_);

      if (info.msg.error == ParseError::kNone) {
        info.status = ReadStatus::kSuccess;

        // Next iteration start after message end.
        start_index_ = (info.msg.data - msg_buf_) + info.msg.len + 1;

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
      if (info.msg.len + kMaxMsgOverhead > kMsgBufLen) {
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

 private:
  static constexpr unsigned int kMsgBufLen = 512;
  static constexpr unsigned int kBytesPerRead = 64;

  // Must be overridden by subclass.
  virtual int ReadBytes(uint8_t *buf, unsigned int len) = 0;
  virtual int WriteBytes(const uint8_t *buf, unsigned int len) = 0;
  virtual uint64_t EpochTimeUs() = 0;

  int FindPreamble(unsigned int start_ind) {
    for (int i = start_ind; i < static_cast<int>(write_index_); ++i) {
      if (msg_buf_[i] == kPreambleVal) {
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

    memmove(msg_buf_, msg_buf_ + start_ind, write_index_ - start_ind);
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

  uint8_t msg_buf_[kMsgBufLen];
  unsigned int write_index_;
  unsigned int start_index_;
  int (*read_func_)(uint8_t *buf, unsigned int len);
};

};  // namespace xbus
