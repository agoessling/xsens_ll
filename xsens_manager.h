#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "msg_id.h"
#include "xbus_parser.h"
#include "xsens_types.h"

namespace xsens {

template <typename T>
static constexpr int ArraySize(const T& array) {
  return sizeof(array) / sizeof(array[0]);
}

template <typename T>
static constexpr int ArraySize() {
  return sizeof(T) / sizeof(T[0]);
}

class XsensManager {
 public:
  enum class ReadStatus {
    kSuccess = 0,  // Parsed message available.
    kNoMsg = 1,  // No message received.
    kReadingMsg = 2,  // Waiting for reception of rest of message.
    kErrorOverflow = 3,  // Receive buffer overflow.
    kErrorMsgTooLarge = 4,  // Received message too large.
    kErrorReadCall = 5,  // Error during user provided read call.
    kErrorParse = 6,  // Error with parsing.  See nested msg.error for further info.
  };

  enum class ConfigResult {
    kSuccess = 0,
    kErrorTimeout = 1,  // Timeout while waiting on acknowledgement
    kErrorPack = 2,  // Error in packing.
    kErrorWriteCall = 3,  // Error during call to WriteBytes.
    kErrorConfig = 4,  // Xsens responded with an error.
    kErrorRead = 5,  // Error during MsgRead.
    kErrorLen = 6,  // Response length incorrect.
    kErrorBadInput = 7,  // Bad input argument.
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

  ConfigResult GetDeviceId(uint32_t& device_id) {
    ConfigResponse resp = SendConfig(MsgId::kReqDID);

    if (resp.result != ConfigResult::kSuccess) return resp.result;

    if (resp.len == 4) {
      device_id = UnpackBigEndian32<uint32_t>(resp.data);
      return ConfigResult::kSuccess;
    }
    if (resp.len == 8) {
      device_id = UnpackBigEndian32<uint32_t>(resp.data + 4);
      return ConfigResult::kSuccess;
    }

    return ConfigResult::kErrorLen;
  }

  ConfigResult GetProductCode(const char *& str, unsigned int& len) {
    ConfigResponse resp = SendConfig(MsgId::kReqProductCode);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len > 20) return ConfigResult::kErrorLen;

    str = reinterpret_cast<const char *>(resp.data);
    len = resp.len;

    return ConfigResult::kSuccess;
  }

  ConfigResult RunSelfTest(bool& passed) {
    SetTimeoutUs(1'000'000);
    ConfigResponse resp = SendConfig(MsgId::kRunSelftest);
    RestoreTimeout();

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 2) return ConfigResult::kErrorLen;

    uint16_t test = UnpackBigEndian16<uint16_t>(resp.data);

    passed = (test & 0x7FF) == 0x7FF;
    return ConfigResult::kSuccess;
  }

  ConfigResult Reset() { return SendConfig(MsgId::kReset).result; }

  ConfigResult GetPortConfig(PortConfigList& config) {
    ConfigResponse resp = SendConfig(MsgId::kReqPortConfig);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 3 * 4) return ConfigResult::kErrorLen;

    for (int i = 0; i < ArraySize(config.array); ++i) {
      config.array[i].protocol = static_cast<Protocol>(resp.data[4 * i + 1] & 0x0F);

      const uint8_t flags = resp.data[4 * i + 2];

      if (flags & (1 << 2)) {
        config.array[i].parity = flags & (1 << 3) ? Parity::kEven : Parity::kOdd;
      } else {
        config.array[i].parity = Parity::kNone;
      }

      config.array[i].stop_bit = flags & (1 << 1) ? StopBit::kTwo : StopBit::kOne;

      config.array[i].flow_control = static_cast<bool>(flags & 0x01);

      config.array[i].baud = static_cast<BaudRate>(resp.data[4 * i + 3]);
    }

    return ConfigResult::kSuccess;
  }

  ConfigResult SetPortConfig(const PortConfigList& config) {
    uint8_t data[3 * 4];
    memset(data, 0, sizeof(data));

    for (int i = 0; i < ArraySize(config.array); ++i) {
      data[4 * i + 1] = static_cast<uint8_t>(config.array[i].protocol);

      uint8_t flags = 0;

      if (config.array[i].parity != Parity::kNone) {
        flags |= (1 << 2);
        flags |= config.array[i].parity == Parity::kEven ? (1 << 3) : 0;
      }

      flags |= config.array[i].stop_bit == StopBit::kTwo ? (1 << 1) : 0;
      flags |= config.array[i].flow_control ? (1 << 0) : 0;

      data[4 * i + 3] = static_cast<uint8_t>(config.array[i].baud);
    }

    return SendConfig(MsgId::kSetPortConfig, data, sizeof(data)).result;
  }

  ConfigResult GetBaudrate(BaudRate& baud) {
    ConfigResponse resp = SendConfig(MsgId::kReqBaudrate);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 1) return ConfigResult::kErrorLen;

    baud = static_cast<BaudRate>(resp.data[0]);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetBaudrate(const BaudRate& baud) {
    const uint8_t byte = static_cast<uint8_t>(baud);
    return SendConfig(MsgId::kSetBaudrate, &byte, 1).result;
  }

  ConfigResult GetOptionFlags(OptionFlags& flags) {
    ConfigResponse resp = SendConfig(MsgId::kReqOptionFlags);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 4) return ConfigResult::kErrorLen;

    flags.raw = UnpackBigEndian32<uint32_t>(resp.data);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetOptionFlags(const OptionFlags& flag) {
    uint8_t data[8];

    const uint32_t set = flag.raw;
    const uint32_t clear = ~flag.raw;

    PackBigEndian32(set, data);
    PackBigEndian32(clear, data + 4);

    return SendConfig(MsgId::kSetOptionFlags, data, sizeof(data)).result;
  }

  ConfigResult RestoreFactoryDefaults() {
    SetTimeoutUs(500'000);
    ConfigResponse resp = SendConfig(MsgId::kRestoreFactoryDef);
    RestoreTimeout();
    return resp.result;
  }

  ConfigResult GetSensorAlignment(Quaternionf& quat) { return GetAlignment(quat, 0); }

  ConfigResult GetLocalAlignment(Quaternionf& quat) { return GetAlignment(quat, 1); }

  ConfigResult SetSensorAlignment(const Quaternionf& quat) { return SetAlignment(quat, 0); }

  ConfigResult SetLocalAlignment(const Quaternionf& quat) { return SetAlignment(quat, 1); }

  ConfigResult GetFilterProfileClassic(FilterType& type) {
    ConfigResponse resp = SendConfig(MsgId::kReqFilterProfile);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 2) return ConfigResult::kErrorLen;

    type = static_cast<FilterType>(UnpackBigEndian16<uint16_t>(resp.data));

    return ConfigResult::kSuccess;
  }

  ConfigResult GetAvailableFilterProfileClassic(FilterProfile (&profile)[5]) {
    ConfigResponse resp = SendConfig(MsgId::kReqAvailableFilterProfiles);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len > 22 * 5 || resp.len % 22) return ConfigResult::kErrorLen;
    const int num_profiles = resp.len / 22;

    for (int i = 0; i < 5; ++i) {
      if (i >= num_profiles) {
        profile[i].type = FilterType::kNone;
        profile[i].version = 0;
        memset(profile[i].label, ' ', 20);
        continue;
      }

      profile[i].type = static_cast<FilterType>(resp.data[22 * i + 0]);
      profile[i].version = resp.data[22 * i + 1];
      for (int j = 0; j < 20; ++j) {
        profile[i].label[j] = resp.data[22 * i + 2 + j];
      }
    }

    return ConfigResult::kSuccess;
  }

  ConfigResult SetFilterProfileClassic(const FilterType& type) {
    uint8_t data[2];
    PackBigEndian16(static_cast<uint16_t>(type), data);

    return SendConfig(MsgId::kSetFilterProfile, data, sizeof(data)).result;
  }

  ConfigResult GetFilterProfile(const char *& str, unsigned int& len) {
    ConfigResponse resp = SendConfig(MsgId::kReqFilterProfile);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    // No check for length here as there seems to be a bug in the MTi-680 where it sometimes sends
    // longer messages.  Limit the actual returned length below.

    str = reinterpret_cast<const char *>(resp.data);
    len = std::min(resp.len, 62U);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetFilterProfile(const char *str, unsigned int len) {
    if (len > 62) return ConfigResult::kErrorBadInput;
    return SendConfig(MsgId::kSetFilterProfile, reinterpret_cast<const uint8_t *>(str), len).result;
  }

  ConfigResult GetGnssPlatform(GnssPlatform& platform) {
    ConfigResponse resp = SendConfig(MsgId::kReqGnssPlatform);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 2) return ConfigResult::kErrorLen;

    platform = static_cast<GnssPlatform>(UnpackBigEndian16<uint16_t>(resp.data));
    return ConfigResult::kSuccess;
  }

  ConfigResult SetGnssPlatform(const GnssPlatform& platform) {
    uint8_t data[2];
    PackBigEndian16(static_cast<uint16_t>(platform), data);

    return SendConfig(MsgId::kSetGnssPlatform, data, sizeof(data)).result;
  }

  ConfigResult GetGnssLeverArm(Vector3f& arm) {
    ConfigResponse resp = SendConfig(MsgId::kReqGnssLeverArm);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 3 * 4) return ConfigResult::kErrorLen;

    arm.x = UnpackBigEndian32<float>(resp.data + 0);
    arm.y = UnpackBigEndian32<float>(resp.data + 4);
    arm.z = UnpackBigEndian32<float>(resp.data + 8);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetGnssLeverArm(const Vector3f& arm) {
    uint8_t data[3 * 4];

    PackBigEndian32(arm.x, data + 0);
    PackBigEndian32(arm.y, data + 4);
    PackBigEndian32(arm.z, data + 8);

    return SendConfig(MsgId::kSetGnssLeverArm, data, sizeof(data)).result;
  }

  ConfigResult GetGnssSettings(GnssSettings& settings) {
    ConfigResponse resp = SendConfig(MsgId::kReqGnssReceiverSettings);
    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 10) return ConfigResult::kErrorLen;

    settings.type = static_cast<GnssType>(UnpackBigEndian16<uint16_t>(resp.data + 0));
    settings.baud = static_cast<BaudRate>(UnpackBigEndian16<uint16_t>(resp.data + 2));
    settings.msg_rate = UnpackBigEndian16<uint16_t>(resp.data + 4);
    settings.options.raw = UnpackBigEndian32<uint32_t>(resp.data + 6);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetGnssSettings(const GnssSettings& settings) {
    uint8_t data[10];

    PackBigEndian16(static_cast<uint16_t>(settings.type), data + 0);
    PackBigEndian16(static_cast<uint16_t>(settings.baud), data + 2);
    PackBigEndian16(settings.msg_rate, data + 4);
    PackBigEndian32(settings.options.raw, data + 6);

    return SendConfig(MsgId::kSetGnssReceiverSettings, data, sizeof(data)).result;
  }

  ConfigResult GetFirmwareRev(FirmwareRev& rev) {
    ConfigResponse resp = SendConfig(MsgId::kReqFWRev);
    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 11) return ConfigResult::kErrorLen;

    rev.major = resp.data[0];
    rev.minor = resp.data[1];
    rev.rev = resp.data[2];
    rev.build_number = UnpackBigEndian32<uint32_t>(resp.data + 3);
    rev.scm_reference = UnpackBigEndian32<uint32_t>(resp.data + 7);

    return ConfigResult::kSuccess;
  }

  // ConfigResult GetOutputConfiguration(DataOutput *data_list, unsigned int len) {
  //   ConfigResponse resp = SendConfig(MsgId::kReqOutputConfiguration);
  //   if (resp.result != ConfigResult::kSuccess) return resp.result;
  //   if (resp.len > 4 * len || resp.len % 4) return ConfigResult::kErrorLen;
  //   const unsigned int num_configs = resp.len / 4;

  //  for (int i = 0; i < num_configs; ++i) {
  //    const uint16_t raw_id = UnpackBigEndian16<uint16_t>(resp.data + 4 * i + 0);

  //    data_list[i].type = static_cast<DataType>(raw_id & 0xFFF0);
  //    data_list[i].precision = static_cast<Precision>(raw_id & 0x03);
  //    data_list[i].coordinates = static_cast<CoordinateSystem>(raw_id & 0xC0);

  //    data_list[i].rate = UnpackBigEndian16<uint16_t>(resp.data + 4 * i + 2);
  //  }

  //  return ConfigResult::kSuccess;
  //}

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

  ConfigResponse SendConfig(MsgId id, const uint8_t *data = nullptr, unsigned int len = 0) {
    ConfigResponse resp;

    PackResult result = PackMsg(write_buf_, kWriteBufLen, id, data, len);

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
        // Response IDs are incremented by one.
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

  ConfigResult GetAlignment(Quaternionf& quat, uint8_t parameter) {
    ConfigResponse resp = SendConfig(MsgId::kReqAlignmentRotation, &parameter, 1);

    if (resp.result != ConfigResult::kSuccess) return resp.result;
    if (resp.len != 1 + 4 * 4) return ConfigResult::kErrorLen;
    if (*resp.data != parameter) return ConfigResult::kErrorConfig;

    quat.w = UnpackBigEndian32<float>(resp.data + 1 + 0);
    quat.x = UnpackBigEndian32<float>(resp.data + 1 + 4);
    quat.y = UnpackBigEndian32<float>(resp.data + 1 + 8);
    quat.z = UnpackBigEndian32<float>(resp.data + 1 + 12);

    return ConfigResult::kSuccess;
  }

  ConfigResult SetAlignment(const Quaternionf& quat, uint8_t parameter) {
    uint8_t data[1 + 4 * 4];

    data[0] = parameter;
    PackBigEndian32(quat.w, data + 1 + 0);
    PackBigEndian32(quat.x, data + 1 + 4);
    PackBigEndian32(quat.y, data + 1 + 8);
    PackBigEndian32(quat.z, data + 1 + 12);

    return SendConfig(MsgId::kSetAlignmentRotation, data, sizeof(data)).result;
  }

  uint8_t read_buf_[kReadBufLen];
  uint8_t write_buf_[kWriteBufLen];
  unsigned int write_index_;
  unsigned int start_index_;
  unsigned int default_timeout_us_;
  unsigned int timeout_us_;
};

};  // namespace xsens
