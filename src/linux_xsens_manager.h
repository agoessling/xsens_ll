#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "src/xsens_manager.h"

namespace xsens {
namespace linux {

static std::ofstream g_stream;

class SerialXsensManager : public XsensManager {
 public:
  static std::unique_ptr<SerialXsensManager> Create(const std::string& port, speed_t baud_rate,
                                                    cc_t timeout_deciseconds) {
    std::unique_ptr<SerialXsensManager> manager(new SerialXsensManager());

    manager->fd_ = open(port.c_str(), O_RDWR);
    if (manager->fd_ < 0) {
      perror("SerialXsensManager");
      return nullptr;
    }

    if (!SetupBlockingSerial(manager->fd_, baud_rate, timeout_deciseconds)) {
      return nullptr;
    }

    g_stream.open("/home/agoessling/raw_bytes.txt");
    if (!g_stream) {
      return nullptr;
    }

    return manager;
  }

  ~SerialXsensManager() { close(fd_); }

  // Not copyable.
  SerialXsensManager(const SerialXsensManager&) = delete;
  SerialXsensManager& operator=(const SerialXsensManager&) = delete;

  std::pair<ConfigResult, std::string> GetProductCode() {
    const char *product_code_raw;
    unsigned int len;

    ConfigResult result = XsensManager::GetProductCode(product_code_raw, len);

    if (result == ConfigResult::kSuccess) {
      // Remove trailing whitespace.
      while (len > 0 && product_code_raw[len - 1] == ' ') len--;

      return std::make_pair(result, std::string(product_code_raw, product_code_raw + len));
    } else {
      return std::make_pair(result, "");
    }
  }

  ConfigResult SetFilterProfile(const std::string& profile) {
    return XsensManager::SetFilterProfile(profile.data(), profile.size());
  }

 private:
  static constexpr int kNumSysCallRetries = 3;
  static constexpr int kTimeoutUs = 100'000;

  SerialXsensManager() : XsensManager(kTimeoutUs), fd_{-1} {}

  static bool SetupBlockingSerial(int fd, speed_t baud_rate, cc_t timeout_deciseconds) {
    if (fd < 0) {
      std::cerr << "Invalid file descriptor: " << fd << std::endl;
      return false;
    }

    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
      perror("SerialXsensManager");
      return false;
    }

    tty.c_cflag &= ~PARENB;  // No parity.
    tty.c_cflag &= ~CSTOPB;  // One stop bit.
    tty.c_cflag &= ~CSIZE;  // Clear size bits.
    tty.c_cflag |= CS8;  // 8 bits.
    tty.c_cflag &= ~CRTSCTS;  // No flow control.
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on read and ignore ctrl lines.

    tty.c_lflag &= ~ICANON;  // Turn off canonical mode. (e.g. Buffer until new line).
    tty.c_lflag &= ~ECHO;  // Disable echo.
    tty.c_lflag &= ~ECHOE;  // Disable erasure.
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo.
    tty.c_lflag &= ~ISIG;  // Disable interpretation of INTR, QUIT and SUSP.

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off software flow ctrl.
    // Disable any special handling of received bytes.
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars).
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed.

    // Blocking with timeout.
    tty.c_cc[VTIME] = timeout_deciseconds;
    tty.c_cc[VMIN] = 0;

    if (cfsetspeed(&tty, baud_rate) != 0) {
      perror("SerialXsensManager");
      return false;
    }

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      perror("SerialXsensManager");
      return false;
    }

    return true;
  }

  int ReadBytes(uint8_t *buf, unsigned int len) override final {
    int retries = kNumSysCallRetries;
    int ret;
    while (retries--) {
      ret = read(fd_, buf, len);
      if (ret >= 0) break;
      perror("read() failed");
    }

    for (unsigned int i = 0; i < ret; ++i) {
      g_stream << std::hex << std::showbase << static_cast<int>(buf[i]) << ",";
      if (i == ret - 1) {
        g_stream << std::endl;
      }
    }

    return ret;
  }

  int FlushBytes() override final {
    int retries = kNumSysCallRetries;
    int ret;
    while (retries--) {
      ret = tcflush(fd_, TCIOFLUSH);
      if (ret >= 0) break;
      perror("tcflush() failed");
    }
    return ret;
  }

  int WriteBytes(const uint8_t *buf, unsigned int len) override final {
    int retries = kNumSysCallRetries;
    int ret;
    while (retries--) {
      ret = write(fd_, buf, len);
      if (ret >= 0) break;
      perror("write() failed");
    }
    return ret;
  }

  uint64_t EpochTimeUs() {
    using namespace std::chrono;

    microseconds epoch_time = duration_cast<microseconds>(system_clock::now().time_since_epoch());
    return epoch_time.count();
  }

  int fd_;
};

};  // namespace linux
};  // namespace xsens
