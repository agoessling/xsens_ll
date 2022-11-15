#pragma once

#include <cstdint>

#include "msg_id.h"

namespace xbus {

static constexpr uint8_t kPreambleVal = 0xFA;
static constexpr uint8_t kBidVal = 0xFF;

enum class MsgError {
  kNone,  // No error
  kPreamble,  // Incorrect preamble
  kBid,  // Incorrect BID
  kLen,  // Incorrect length
  kChecksum,  // Incorrect checksum
};

struct ParsedMsg {
  MsgError error;
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
  ParsedMsg msg = {
      .len = 0,
      .data = nullptr,
  };

  // Minimum message length.
  if (len < 5) {
    msg.error = MsgError::kLen;
    return msg;
  }

  if (buf[0] != kPreambleVal) {
    msg.error = MsgError::kPreamble;
    return msg;
  }

  if (buf[1] != kBidVal) {
    msg.error = MsgError::kBid;
    return msg;
  }

  msg.id = static_cast<MsgId>(buf[2]);

  unsigned int header_len = 4;

  // Extended length message.
  if (buf[3] == 0xFF) {
    header_len = 6;

    if (len < 7) {
      msg.error = MsgError::kLen;
      return msg;
    }

    msg.len = BigEndian16<unsigned int>(&buf[4]);
  } else {
    msg.len = buf[3];
  }

  // Check total message length.
  if (len < header_len + msg.len + 1) {
    msg.error = MsgError::kLen;
    return msg;
  }

  msg.data = buf + header_len;

  uint8_t checksum = 0;
  for (unsigned int i = 1; i < len; ++i) {
    checksum += buf[i];
  }

  if (checksum != 0) {
    msg.error = MsgError::kChecksum;
    return msg;
  }

  msg.error = MsgError::kNone;

  return msg;
}

};  // namespace xbus
