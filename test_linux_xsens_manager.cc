#include <bitset>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <argparse.hpp>

#include "data_packet.h"
#include "linux_xsens_manager.h"
#include "xsens_types.h"

using namespace xsens;
using namespace xsens::linux;
using namespace xsens::data;

static inline void CheckOrExit(const XsensManager::ConfigResult& result) {
  if (result != XsensManager::ConfigResult::kSuccess) {
    std::cerr << "Config command not successful. ConfigResult = ";
    std::cerr << static_cast<unsigned int>(result) << std::endl;
    std::exit(-1);
  }
}

static constexpr BaudRate kNewBaud = BaudRate::k921600;

int main(int argc, char **argv) {
  argparse::ArgumentParser program("test_linux_xsens_manager");
  program.add_argument("-p", "--port").required().help("Serial port.");
  program.add_argument("-b", "--baudrate").scan<'i', int>().required().help("Serial baud rate.");
  program.add_argument("--mti-8").default_value(false).implicit_value(true).help("Test Mti-8.");
  program.add_argument("--defaults")
      .default_value(false)
      .implicit_value(true)
      .help("Restore to factory defaults.");

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(-1);
  }

  std::unique_ptr<SerialXsensManager> manager = SerialXsensManager::Create(
      program.get<std::string>("--port"), program.get<int>("--baudrate"), 1);

  if (!manager) {
    std::cerr << "Could not create manager." << std::endl;
    std::exit(-1);
  }

  std::cout << "Going to Config mode. ";
  CheckOrExit(manager->GoToConfig());
  std::cout << std::endl;

  std::cout << "Device ID: ";
  uint32_t device_id;
  CheckOrExit(manager->GetDeviceId(device_id));
  std::cout << std::hex << std::showbase << device_id << std::dec << std::endl;

  std::cout << "Product Code: ";
  const char *product_code_raw;
  unsigned int len;
  CheckOrExit(manager->GetProductCode(product_code_raw, len));
  std::string product_code(product_code_raw, product_code_raw + len);
  std::cout << product_code << std::endl;

  std::cout << "Firmware Revision: ";
  FirmwareRev firmware;
  CheckOrExit(manager->GetFirmwareRev(firmware));
  std::cout << std::endl;
  std::cout << "  Revision: " << static_cast<int>(firmware.major) << ".";
  std::cout << static_cast<int>(firmware.minor) << "." << static_cast<int>(firmware.rev);
  std::cout << std::endl;
  std::cout << "  Build Number: " << firmware.build_number << std::endl;
  std::cout << "  SCM Reference: " << firmware.scm_reference << std::endl;

  std::cout << "Self test: " << std::flush;
  bool self_test;
  CheckOrExit(manager->RunSelfTest(self_test));
  std::cout << (self_test ? "PASS" : "FAIL") << std::endl;

  if (!program.get<bool>("--mti-8")) {
    std::cout << "Port Config: " << std::flush;
    PortConfigList config;
    CheckOrExit(manager->GetPortConfig(config));
    std::cout << std::endl;

    std::cout << "  Protocol: " << std::bitset<4>(static_cast<int>(config.host_uart.protocol));
    std::cout << std::endl;
    std::cout << "  Parity: " << static_cast<int>(config.host_uart.parity) << std::endl;
    std::cout << "  Stop Bit: " << static_cast<int>(config.host_uart.stop_bit) << std::endl;
    std::cout << "  Flow Control: " << config.host_uart.flow_control << std::endl;
    std::cout << "  Baud Rate: " << std::hex << std::showbase;
    std::cout << static_cast<int>(config.host_uart.baud) << std::dec << std::endl;

    std::cout << "Setting baud. ";
    config.host_uart.baud = kNewBaud;
    CheckOrExit(manager->SetPortConfig(config));
    std::cout << std::endl;
  }

  std::cout << "Get Baudrate: " << std::flush;
  BaudRate baud;
  CheckOrExit(manager->GetBaudrate(baud));
  std::cout << std::hex << std::showbase << static_cast<int>(baud) << std::dec << std::endl;

  std::cout << "Setting Baudrate." << std::flush;
  baud = kNewBaud;
  CheckOrExit(manager->SetBaudrate(baud));
  std::cout << std::endl;

  std::cout << "Get option flags: " << std::flush;
  OptionFlags flags;
  CheckOrExit(manager->GetOptionFlags(flags));
  std::cout << std::endl;

  std::cout << "  disable_auto_store: " << flags.disable_auto_store << std::endl;
  std::cout << "  disable_auto_measurement: " << flags.disable_auto_measurement << std::endl;
  std::cout << "  enable_beidou: " << flags.enable_beidou << std::endl;
  std::cout << "  reserved0: " << flags.reserved0 << std::endl;
  std::cout << "  enable_ahs: " << flags.enable_ahs << std::endl;
  std::cout << "  enable_orientation_smoother: " << flags.enable_orientation_smoother << std::endl;
  std::cout << "  enable_configurable_bus_id: " << flags.enable_configurable_bus_id << std::endl;
  std::cout << "  enable_inrun_compass_calibration: " << flags.enable_inrun_compass_calibration
            << std::endl;
  std::cout << "  reserved1: " << flags.reserved1 << std::endl;
  std::cout << "  enable_config_message_at_startup: " << flags.enable_config_message_at_startup
            << std::endl;
  std::cout << "  reserved2: " << flags.reserved2 << std::endl;
  std::cout << "  enable_position_velocity_smoother: " << flags.enable_position_velocity_smoother
            << std::endl;
  std::cout << "  enable_continuous_zru: " << flags.enable_continuous_zru << std::endl;
  std::cout << "  reserved3: " << flags.reserved3 << std::endl;

  std::cout << "Setting option flags." << std::flush;
  flags.disable_auto_measurement = true;
  CheckOrExit(manager->SetOptionFlags(flags));
  std::cout << std::endl;

  std::cout << "Get sensor rotation: " << std::flush;
  Quaternionf quat;
  CheckOrExit(manager->GetSensorAlignment(quat));
  std::cout << std::endl;
  std::cout << "  w: " << quat.w << std::endl;
  std::cout << "  x: " << quat.x << std::endl;
  std::cout << "  y: " << quat.y << std::endl;
  std::cout << "  z: " << quat.z << std::endl;

  std::cout << "Get local rotation: " << std::flush;
  CheckOrExit(manager->GetLocalAlignment(quat));
  std::cout << std::endl;
  std::cout << "  w: " << quat.w << std::endl;
  std::cout << "  x: " << quat.x << std::endl;
  std::cout << "  y: " << quat.y << std::endl;
  std::cout << "  z: " << quat.z << std::endl;

  std::cout << "Setting sensor rotation. " << std::flush;
  CheckOrExit(manager->SetSensorAlignment(quat));
  std::cout << std::endl;

  std::cout << "Setting local rotation. " << std::flush;
  CheckOrExit(manager->SetLocalAlignment(quat));
  std::cout << std::endl;

  std::cout << "Get filter profiles: " << std::flush;
  FilterProfile profile[5];
  CheckOrExit(manager->GetAvailableFilterProfileClassic(profile));
  std::cout << std::endl;
  for (int i = 0; i < 5; ++i) {
    std::cout << "  [" << i << "]:" << std::endl;
    std::cout << "    Type: " << static_cast<int>(profile[i].type) << std::endl;
    std::cout << "    Version: " << static_cast<int>(profile[i].version) << std::endl;

    std::string label(profile[i].label, profile[i].label + 20);
    std::cout << "    Label: " << label << std::endl;
  }

  if (program.get<bool>("--mti-8")) {
    std::cout << "Get filter profile: " << std::flush;
    FilterType filter_type;
    CheckOrExit(manager->GetFilterProfileClassic(filter_type));
    std::cout << static_cast<int>(filter_type) << std::endl;

    std::cout << "Setting filter profile. " << std::flush;
    CheckOrExit(manager->SetFilterProfileClassic(FilterType::kMti8GeneralRtk));
    std::cout << std::endl;
  } else {
    std::cout << "Get filter profile: " << std::flush;
    const char *str;
    unsigned int len;
    CheckOrExit(manager->GetFilterProfile(str, len));
    std::string label(str, str + len);
    std::cout << label << std::endl;

    std::cout << "Setting filter profile. " << std::flush;
    std::string filter = "General_RTK";
    CheckOrExit(manager->SetFilterProfile(filter.data(), filter.size()));
    std::cout << std::endl;
  }

  std::cout << "Get GNSS Platform: " << std::flush;
  GnssPlatform platform;
  CheckOrExit(manager->GetGnssPlatform(platform));
  std::cout << static_cast<int>(platform) << std::endl;

  std::cout << "Setting GNSS platform. " << std::flush;
  CheckOrExit(manager->SetGnssPlatform(GnssPlatform::kAirborneLt2g));
  std::cout << std::endl;

  std::cout << "Get GNSS Lever Arm: " << std::flush;
  Vector3f lever_arm;
  CheckOrExit(manager->GetGnssLeverArm(lever_arm));
  std::cout << std::endl;
  std::cout << "  x: " << lever_arm.x << std::endl;
  std::cout << "  y: " << lever_arm.y << std::endl;
  std::cout << "  z: " << lever_arm.z << std::endl;

  std::cout << "Setting GNSS lever arm. " << std::flush;
  lever_arm.x = 0.1;
  CheckOrExit(manager->SetGnssLeverArm(lever_arm));
  std::cout << std::endl;

  std::cout << "Get GNSS settings: " << std::flush;
  GnssSettings gnss_settings;
  CheckOrExit(manager->GetGnssSettings(gnss_settings));
  std::cout << std::endl;
  std::cout << "  Type: " << static_cast<int>(gnss_settings.type) << std::endl;
  std::cout << "  Baud: " << std::hex << static_cast<int>(gnss_settings.baud) << std::dec;
  std::cout << std::endl;
  std::cout << "  Message Rate: " << gnss_settings.msg_rate << std::endl;
  std::cout << "  Options: " << gnss_settings.options.raw << std::endl;

  std::cout << "Set GNSS settings. " << std::flush;
  gnss_settings.baud = BaudRate::k921600;
  CheckOrExit(manager->SetGnssSettings(gnss_settings));
  std::cout << std::endl;

  if (program.get<bool>("--defaults")) {
    std::cout << "Restoring to factory defaults. ";
    CheckOrExit(manager->RestoreFactoryDefaults());
    std::cout << std::endl;
  }

  std::cout << "Resetting. ";
  CheckOrExit(manager->Reset());
  std::cout << std::endl;

  std::exit(0);
}
