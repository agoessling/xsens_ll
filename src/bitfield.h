#pragma once

namespace xsens {
namespace util {

template <typename T>
T GetField(const T raw, const unsigned int offset, const unsigned int len) {
  const T mask = (1 << len) - 1;
  T field = (raw >> offset) & mask;
  return field;
}

// Unpack and Pack used for sequentially editing bitfields. `offset` is updated to next offset.
template <typename T>
T UnpackField(const T raw, unsigned int& offset, const unsigned int len) {
  const T mask = (1 << len) - 1;
  T field = (raw >> offset) & mask;
  offset += len;
  return field;
}

template <typename T>
void PackField(T& raw, const T field, unsigned int& offset, const unsigned int len) {
  const T mask = (1 << len) - 1;
  raw &= ~(mask << offset);
  raw |= (field & mask) << offset;
  offset += len;
}

};  // namespace util
};  // namespace xsens
