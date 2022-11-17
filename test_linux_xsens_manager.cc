#include <cstdlib>
#include <memory>

#include <argparse.hpp>

#include "linux_xsens_manager.h"

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

  using namespace xbus;
  using namespace xbus::linux;

  std::unique_ptr<SerialXsensManager> manager = SerialXsensManager::Create(
      program.get<std::string>("--port"), program.get<int>("--baudrate"), 1);

  if (!manager) {
    std::cerr << "Could not create manager." << std::endl;
    std::exit(-1);
  }

  XsensManager::ConfigResult result = manager->GoToConfig();
  if (result != XsensManager::ConfigResult::kSuccess) {
    std::cerr << "GoToConfig not successful. ConfigResult = " << static_cast<unsigned int>(result);
    std::cerr << std::endl;
    std::exit(-1);
  }

  std::exit(0);
}
