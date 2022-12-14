#include <bitset>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <argparse.hpp>

#include "src/data_packet.h"
#include "src/linux_xsens_manager.h"
#include "src/xsens_types.h"

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
  program.add_argument("--run").scan<'f', double>().help("Take measurements for duration.");
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
  auto [result, product_code] = manager->GetProductCode();
  CheckOrExit(result);
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
    CheckOrExit(manager->SetFilterProfile("General_RTK"));
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
  gnss_settings.type = GnssType::kUbloxZedF9p;
  gnss_settings.baud = BaudRate::k230400;
  gnss_settings.msg_rate = 5;
  gnss_settings.options.ublox = GnssPlatform::kAirborneLt2g;
  CheckOrExit(manager->SetGnssSettings(gnss_settings));
  std::cout << std::endl;

  OutputConfig configs[] = {
      {.type_id = PacketCounter::id, .rate = 100},
      {.type_id = EulerAngles<Float32, Ned>::id, .rate = 100},
      {.type_id = StatusWord::id, .rate = 100},
      {.type_id = UtcTime::id, .rate = 100},
  };

  std::cout << "Setting output configs. " << std::flush;
  CheckOrExit(manager->SetOutputConfiguration(configs, sizeof(configs) / sizeof(configs[0])));
  std::cout << std::endl;

  if (auto run_time = program.present<double>("--run")) {
    std::cout << "Going to measurement. " << std::flush;
    CheckOrExit(manager->GoToMeasurement());
    std::cout << std::endl;

    using namespace std::chrono;
    time_point alarm = system_clock::now() + duration<double>(*run_time);
    while (system_clock::now() < alarm) {
      XsensManager::MsgInfo info = manager->ReadMsg();

      if (info.status != XsensManager::ReadStatus::kSuccess) continue;
      if (info.msg.id != MsgId::kMtData2) continue;

      auto euler = GetData<EulerAngles<Float32, Ned>>(info.msg.data, info.msg.len);
      auto status = GetData<StatusWord>(info.msg.data, info.msg.len);
      auto utc_time = GetData<UtcTime>(info.msg.data, info.msg.len);

      if (!euler || !status || !utc_time) {
        std::cout << "Output data not present in MTData2." << std::endl;
        continue;
      }

      std::cout << static_cast<int>(utc_time->year) << "-" << static_cast<int>(utc_time->month)
                << "-" << static_cast<int>(utc_time->day) << " " << static_cast<int>(utc_time->hour)
                << ":" << static_cast<int>(utc_time->minute) << ":"
                << static_cast<int>(utc_time->second) << std::endl;

      std::cout << "Roll: " << euler->x << std::endl;
      std::cout << "Pitch: " << euler->y << std::endl;
      std::cout << "Yaw: " << euler->z << std::endl;

      std::cout << "Status Word:" << std::endl;
      std::cout << "  self_test: " << static_cast<int>(status->status_byte.self_test) << std::endl;
      std::cout << "  filter_valid: " << static_cast<int>(status->status_byte.filter_valid)
                << std::endl;
      std::cout << "  gnss_fix: " << static_cast<int>(status->status_byte.gnss_fix) << std::endl;
      std::cout << "  no_rot: " << static_cast<int>(status->status_byte.no_rotation_status())
                << std::endl;
      std::cout << "  rep_mo: " << static_cast<int>(status->status_byte.rep_mo) << std::endl;
      std::cout << "  clock_sync: " << static_cast<int>(status->status_byte.clock_sync)
                << std::endl;

      std::cout << std::endl;
    }

    std::cout << "Going to config. " << std::flush;
    CheckOrExit(manager->GoToConfig());
    std::cout << std::endl;
  }

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
