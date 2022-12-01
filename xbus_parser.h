#pragma once

#include <cstdint>

#include "msg_id.h"

namespace xsens {

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
T UnpackBigEndian16(const uint8_t *buf) {
  static_assert(sizeof(T) == 2);
  uint16_t raw_val = (static_cast<uint16_t>(buf[0]) << 8) |
                     (static_cast<uint16_t>(buf[1]) << 0);
  return reinterpret_cast<T&>(raw_val);
}

template <typename T>
T UnpackBigEndian32(const uint8_t *buf) {
  static_assert(sizeof(T) == 4);
  uint32_t raw_val = (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) |
                     (static_cast<uint32_t>(buf[2]) << 8) | (static_cast<uint32_t>(buf[3]) << 0);
  return reinterpret_cast<T&>(raw_val);
}

template <typename T>
T UnpackBigEndian64(const uint8_t *buf) {
  static_assert(sizeof(T) == 8);
  uint64_t raw_val = (static_cast<uint64_t>(buf[0]) << 56) | (static_cast<uint64_t>(buf[1]) << 48) |
                     (static_cast<uint64_t>(buf[2]) << 40) | (static_cast<uint64_t>(buf[3]) << 32) |
                     (static_cast<uint64_t>(buf[4]) << 24) | (static_cast<uint64_t>(buf[5]) << 16) |
                     (static_cast<uint64_t>(buf[6]) << 8) | (static_cast<uint64_t>(buf[7]) << 0);
  return reinterpret_cast<T&>(raw_val);
}

template <typename T>
void PackBigEndian16(T data, uint8_t *buf) {
  static_assert(sizeof(data) == 2);
  const uint16_t raw_data = reinterpret_cast<uint16_t&>(data);
  buf[0] = static_cast<uint8_t>(raw_data >> 8);
  buf[1] = static_cast<uint8_t>(raw_data >> 0);
}

template <typename T>
void PackBigEndian32(T data, uint8_t *buf) {
  static_assert(sizeof(data) == 4);
  const uint32_t raw_data = reinterpret_cast<uint32_t&>(data);
  buf[0] = static_cast<uint8_t>(raw_data >> 24);
  buf[1] = static_cast<uint8_t>(raw_data >> 16);
  buf[2] = static_cast<uint8_t>(raw_data >> 8);
  buf[3] = static_cast<uint8_t>(raw_data >> 0);
}

template <typename T>
void PackBigEndian64(T data, uint8_t *buf) {
  static_assert(sizeof(data) == 8);
  const uint64_t raw_data = reinterpret_cast<uint64_t&>(data);
  buf[0] = static_cast<uint8_t>(raw_data >> 56);
  buf[1] = static_cast<uint8_t>(raw_data >> 48);
  buf[2] = static_cast<uint8_t>(raw_data >> 40);
  buf[3] = static_cast<uint8_t>(raw_data >> 32);
  buf[4] = static_cast<uint8_t>(raw_data >> 24);
  buf[5] = static_cast<uint8_t>(raw_data >> 16);
  buf[6] = static_cast<uint8_t>(raw_data >> 8);
  buf[7] = static_cast<uint8_t>(raw_data >> 0);
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

    msg.len = UnpackBigEndian16<uint16_t>(&buf[4]);
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

enum class PackError {
  kNone,
  kOverflow,  // Buffer not big enough for packed message.
  kDataTooLarge,  // Data packet too large for xbus format.
};

struct PackResult {
  PackError error;
  unsigned int len;
};

PackResult PackMsg(uint8_t *buf, unsigned int buf_len, MsgId id, const uint8_t *data,
                   unsigned int data_len) {
  PackResult result = {.error = PackError::kNone, .len = 0};

  if (data_len > UINT16_MAX) {
    result.error = PackError::kDataTooLarge;
    return result;
  }

  const bool extended = data_len > 254;
  const unsigned int header_len = extended ? 6 : 4;
  const unsigned int packed_len = header_len + data_len + 1;

  if (packed_len > buf_len) {
    result.error = PackError::kOverflow;
    return result;
  }

  buf[0] = kPreambleVal;
  buf[1] = kBidVal;
  buf[2] = static_cast<uint8_t>(id);

  if (extended) {
    buf[3] = 0xFF;
    PackBigEndian16(static_cast<uint16_t>(data_len), &buf[4]);
  } else {
    buf[3] = static_cast<uint8_t>(data_len);
  }

  for (unsigned int i = 0; i < data_len; ++i) {
    buf[header_len + i] = data[i];
  }

  uint8_t checksum = 0;
  for (unsigned int i = 1; i < packed_len - 1; ++i) {
    checksum -= buf[i];
  }

  buf[packed_len - 1] = checksum;

  result.len = packed_len;
  return result;
}

};  // namespace xsens
