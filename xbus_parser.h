#pragma once

#include <cstdint>

#include "msg_id.h"

namespace xbus {

static constexpr int kMaxMsgOverhead = 7;  // Maximum bytes of overhead.
static constexpr uint8_t kPreambleVal = 0xFA;
static constexpr uint8_t kBidVal = 0xFF;

enum class ParseError {
  kNone,  // No error
  kPreamble,  // Incorrect preamble
  kBid,  // Incorrect BID
  kLen,  // Insufficient length
  kChecksum,  // Incorrect checksum
};

struct ParsedMsg {
  ParseError error;
  MsgId id;
  unsigned int len;
  const uint8_t *data;
};

template <typename T>
T BigEndian16(const uint8_t *buf) {
  return static_cast<T>((buf[0] << 8U) | buf[1]);
}

template <typename T>
T BigEndian32(const uint8_t *buf) {
  return static_cast<T>((buf[0] << 24U) | (buf[1] << 16U) | (buf[2] << 8U) | buf[3]);
}

ParsedMsg ParseMsg(const uint8_t *buf, unsigned int len) {
  ParsedMsg msg = {.len = 0, .data = nullptr};

  // Minimum message length.
  if (len < 5) {
    msg.error = ParseError::kLen;
    return msg;
  }

  if (buf[0] != kPreambleVal) {
    msg.error = ParseError::kPreamble;
    return msg;
  }

  if (buf[1] != kBidVal) {
    msg.error = ParseError::kBid;
    return msg;
  }

  msg.id = static_cast<MsgId>(buf[2]);

  unsigned int header_len = 4;

  // Extended length message.
  if (buf[3] == 0xFF) {
    header_len = 6;

    if (len < 7) {
      msg.error = ParseError::kLen;
      return msg;
    }

    msg.len = BigEndian16<unsigned int>(&buf[4]);
  } else {
    msg.len = buf[3];
  }

  const unsigned int total_msg_len = header_len + msg.len + 1;

  // Check total message length.
  if (len < total_msg_len) {
    msg.error = ParseError::kLen;
    return msg;
  }

  msg.data = buf + header_len;

  uint8_t checksum = 0;
  for (unsigned int i = 1; i < total_msg_len; ++i) {
    checksum += buf[i];
  }

  if (checksum != 0) {
    msg.error = ParseError::kChecksum;
    return msg;
  }

  msg.error = ParseError::kNone;

  return msg;
}

};  // namespace xbus
