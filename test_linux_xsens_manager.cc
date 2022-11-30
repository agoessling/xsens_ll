#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <argparse.hpp>

#include "linux_xsens_manager.h"

using namespace xbus;
using namespace xbus::linux;

static inline void CheckOrExit(const XsensManager::ConfigResult& result) {
  if (result != XsensManager::ConfigResult::kSuccess) {
    std::cerr << "Config command not successful. ConfigResult = ";
    std::cerr << static_cast<unsigned int>(result) << std::endl;
    std::exit(-1);
  }
}

int main(int argc, char **argv) {
  argparse::ArgumentParser program("test_linux_xsens_manager");
  program.add_argument("-p", "--port").required().help("Serial port.");
  program.add_argument("-b", "--baudrate").scan<'i', int>().required().help("Serial baud rate.");

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

  CheckOrExit(manager->GoToConfig());

  uint32_t device_id;
  CheckOrExit(manager->GetDeviceId(&device_id));

  std::cout << "Device ID: " << std::hex << std::showbase << device_id << std::endl;

  const char *product_code_raw;
  unsigned int len;
  CheckOrExit(manager->GetProductCode(&product_code_raw, &len));

  std::string product_code(product_code_raw, product_code_raw + len);
  std::cout << "Product Code: " << product_code << std::endl;

  bool self_test;
  CheckOrExit(manager->RunSelfTest(&self_test));
  std::cout << "Self test: " << (self_test ? "PASS" : "FAIL") << std::endl;

  std::exit(0);
}
